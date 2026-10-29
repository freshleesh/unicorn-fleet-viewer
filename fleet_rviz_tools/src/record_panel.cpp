// Fleet recorder panel: one button to record what the cars send to this PC.
// Folder (chosen with "..."), bag name (empty = fleet_<MMDD_HHMM>), REC / STOP.
// Runs `ros2 bag record -s mcap --max-bag-duration 600 -o <folder>/<name>` with each car's pose
// (/<car>/odom), motor command (/<car>/drive), camera (/<car>/image/compressed) and /race/state,
// in this RViz's environment (fleet domain). STOP sends SIGINT so the bag is closed cleanly;
// closing RViz while recording also stops it cleanly.
#include <signal.h>

#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/panel.hpp"

namespace fleet_rviz_tools
{

class RecordPanel : public rviz_common::Panel
{
public:
  explicit RecordPanel(QWidget * parent = nullptr) : rviz_common::Panel(parent)
  {
    folder_ = new QLineEdit(QDir::homePath() + "/bag");
    auto * browse = new QPushButton("...");
    browse->setMaximumWidth(36);
    name_ = new QLineEdit;
    name_->setPlaceholderText("fleet_MMDD_HHMM");
    button_ = new QPushButton("REC");
    button_->setMinimumHeight(40);
    status_ = new QLabel("ready");
    status_->setWordWrap(true);

    auto * grid = new QGridLayout;
    grid->addWidget(new QLabel("Folder"), 0, 0);
    grid->addWidget(folder_, 0, 1);
    grid->addWidget(browse, 0, 2);
    grid->addWidget(new QLabel("Name"), 1, 0);
    grid->addWidget(name_, 1, 1, 1, 2);
    grid->addWidget(button_, 2, 0, 1, 3);
    grid->addWidget(status_, 3, 0, 1, 3);
    grid->setColumnStretch(1, 1);
    auto * layout = new QVBoxLayout;
    layout->addLayout(grid);
    layout->addStretch();
    setLayout(layout);
    idleStyle();

    connect(browse, &QPushButton::clicked, this, [this] {
      const QString dir = QFileDialog::getExistingDirectory(this, "Bag folder", folder_->text());
      if (!dir.isEmpty()) {
        folder_->setText(dir);
        Q_EMIT configChanged();
      }
    });
    connect(folder_, &QLineEdit::editingFinished, this, [this] { Q_EMIT configChanged(); });
    connect(button_, &QPushButton::clicked, this, [this] { running() ? stop() : start(); });

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { tick(); });
  }

  ~RecordPanel() override { stop(); }

  void save(rviz_common::Config config) const override
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Folder", folder_->text());
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    QString f;
    if (config.mapGetString("Folder", &f) && !f.isEmpty()) {
      folder_->setText(f);
    }
  }

private:
  bool running() const { return proc_ && proc_->state() != QProcess::NotRunning; }

  void start()
  {
    QString dir = folder_->text().trimmed();
    if (dir.startsWith("~")) {
      dir = QDir::homePath() + dir.mid(1);
    }
    if (dir.isEmpty() || !QDir().mkpath(dir)) {
      status_->setText("<font color='#c62828'>cannot use folder: " + dir.toHtmlEscaped() + "</font>");
      return;
    }
    QString name = name_->text().trimmed();
    if (name.isEmpty()) {
      name = "fleet_" + QDateTime::currentDateTime().toString("MMdd_HHmm");
    }
    out_ = QDir(dir).filePath(name);
    for (int i = 2; QFileInfo::exists(out_); ++i) {          // ros2 bag refuses an existing folder
      out_ = QDir(dir).filePath(name + "_" + QString::number(i));
    }
    QStringList args{"bag", "record", "-s", "mcap", "--max-bag-duration", "600", "-o", out_, "/race/state"};
    for (const char * car : {"nuc1", "nuc4", "nuc5"}) {
      for (const char * t : {"odom", "drive", "image/compressed"}) {
        args << QString("/%1/%2").arg(car, t);
      }
    }
    proc_ = new QProcess(this);
    proc_->setProcessChannelMode(QProcess::MergedChannels);
    connect(proc_, &QProcess::readyRead, this, [this] {
      const QStringList lines = QString::fromUtf8(proc_->readAll()).split('\n', Qt::SkipEmptyParts);
      if (!lines.isEmpty()) {
        last_line_ = lines.last().trimmed();
      }
    });
    connect(proc_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
      [this](int code, QProcess::ExitStatus) {
        timer_->stop();
        idleStyle();
        if (stopping_) {
          status_->setText("saved: " + out_.toHtmlEscaped() + " (" + sizeText() + ")");
        } else {
          status_->setText("<font color='#c62828'>recorder stopped (exit " + QString::number(code) + "): " +
                           last_line_.toHtmlEscaped() + "</font>");
        }
        stopping_ = false;
      });
    stopping_ = false;
    last_line_.clear();
    proc_->start("ros2", args);
    if (!proc_->waitForStarted(3000)) {
      status_->setText("<font color='#c62828'>could not start ros2 bag (is ROS sourced?)</font>");
      return;
    }
    started_ = QDateTime::currentDateTime();
    button_->setText("STOP");
    button_->setStyleSheet("background-color:#c62828; color:white; font-weight:bold;");
    folder_->setEnabled(false);
    name_->setEnabled(false);
    timer_->start(1000);
    tick();
  }

  void stop()
  {
    if (!running()) {
      return;
    }
    stopping_ = true;
    ::kill(static_cast<pid_t>(proc_->processId()), SIGINT);   // let rosbag2 close the file
    if (!proc_->waitForFinished(15000)) {
      proc_->kill();
      proc_->waitForFinished(2000);
    }
  }

  void tick()
  {
    const qint64 s = started_.secsTo(QDateTime::currentDateTime());
    status_->setText(QString("<font color='#c62828'>&#9679; REC</font> %1:%2 &middot; %3<br>%4")
                     .arg(s / 60, 2, 10, QChar('0')).arg(s % 60, 2, 10, QChar('0'))
                     .arg(sizeText(), out_.toHtmlEscaped()));
  }

  QString sizeText() const
  {
    qint64 bytes = 0;
    QDirIterator it(out_, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
      it.next();
      bytes += it.fileInfo().size();
    }
    return QString::number(bytes / 1048576.0, 'f', 1) + " MB";
  }

  void idleStyle()
  {
    button_->setText("REC");
    button_->setStyleSheet("font-weight:bold;");
    folder_->setEnabled(true);
    name_->setEnabled(true);
  }

  QLineEdit * folder_;
  QLineEdit * name_;
  QPushButton * button_;
  QLabel * status_;
  QTimer * timer_;
  QProcess * proc_ = nullptr;
  QString out_;
  QString last_line_;
  QDateTime started_;
  bool stopping_ = false;
};

}  // namespace fleet_rviz_tools

PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::RecordPanel, rviz_common::Panel)
