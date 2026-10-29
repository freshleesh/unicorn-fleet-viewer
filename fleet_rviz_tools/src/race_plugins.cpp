// Race plugins, all talking to fleet_viewer's lap_timer.py:
//   StartLineTool  click-drag on the track: where the start/finish line is and which way the race goes
//                  -> /race/start_line (PoseStamped)
//   RacePanel      main view: lap count, START (3 s countdown) / RESET -> /race/command, standings table
//   RaceHud        driver screen: big position / lap / lap time for one car (config key "Car")
// /race/state is JSON (std_msgs/String). Widgets are refreshed from a Qt timer, never from the
// ROS callback, and the running lap time is extrapolated locally so it ticks smoothly.
#include <chrono>
#include <map>
#include <mutex>

#include <QColor>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPainter>
#include <QPalette>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_default_plugins/tools/goal_pose/goal_tool.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "std_msgs/msg/string.hpp"

namespace fleet_rviz_tools
{

namespace
{
const std::vector<std::pair<QString, QString>> kCars{
  {"nuc1", "#ff6419"}, {"nuc4", "#3296ff"}, {"nuc5", "#4bff4b"}};

QString carColor(const QString & car)
{
  for (const auto & [c, col] : kCars) {
    if (c == car) {
      return col;
    }
  }
  return "#ffffff";
}

QString fmtTime(const QJsonValue & v)
{
  if (!v.isDouble()) {
    return "--";
  }
  const double t = v.toDouble();
  const int m = static_cast<int>(t / 60.0);
  return QString("%1:%2").arg(m).arg(t - 60.0 * m, 5, 'f', 2, QChar('0'));
}

double nowSec()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Latest /race/state, written by the ROS callback, read by the Qt timer.
class RaceStateSub
{
public:
  void subscribe(rclcpp::Node::SharedPtr node)
  {
    sub_ = node->create_subscription<std_msgs::msg::String>(
      "/race/state", 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        json_ = QString::fromStdString(msg->data);
        stamp_ = nowSec();
      });
  }
  // returns false when no state arrived for `timeout` s; `age` = seconds since it arrived
  bool latest(QJsonObject * state, double * age, double timeout = 2.0)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    *age = nowSec() - stamp_;
    if (json_.isEmpty() || *age > timeout) {
      return false;
    }
    *state = QJsonDocument::fromJson(json_.toUtf8()).object();
    return true;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::mutex mutex_;
  QString json_;
  double stamp_{0.0};
};
}  // namespace

// ---------------------------------------------------------------- start line tool

class StartLineTool : public rviz_default_plugins::tools::GoalTool
{
public:
  explicit StartLineTool(QString car = "") : car_(car) {}
  void onInitialize() override
  {
    GoalTool::onInitialize();
    setName(car_.isEmpty() ? "Start Line" : "Start: " + car_);
    getPropertyContainer()->subProp("Topic")->setValue(
      car_.isEmpty() ? "/race/start_line" : "/race/" + car_ + "/start_line");
    const QColor color(car_.isEmpty() ? "#ffd700" : carColor(car_));
    arrow_->setColor(color.redF(), color.greenF(), color.blueF(), 1.0f);
    QPixmap icon = rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetGoal.png");
    QPainter p(&icon);
    p.setCompositionMode(QPainter::CompositionMode_SourceIn);
    p.fillRect(icon.rect(), color);
    p.end();
    setIcon(QIcon(icon));
  }
private:
  QString car_;
};
class StartLineNuc1 : public StartLineTool { public: StartLineNuc1() : StartLineTool("nuc1") {} };
class StartLineNuc4 : public StartLineTool { public: StartLineNuc4() : StartLineTool("nuc4") {} };
class StartLineNuc5 : public StartLineTool { public: StartLineNuc5() : StartLineTool("nuc5") {} };

// ---------------------------------------------------------------- race control panel (main view)

class RacePanel : public rviz_common::Panel
{
public:
  explicit RacePanel(QWidget * parent = nullptr) : rviz_common::Panel(parent)
  {
    laps_ = new QSpinBox;
    laps_->setRange(1, 99);
    laps_->setValue(3);
    laps_->setSuffix(" laps");
    auto * start = new QPushButton("START");
    auto * reset = new QPushButton("RESET");
    start->setStyleSheet("background-color:#2e7d32; color:white; font-weight:bold;");
    start->setMinimumHeight(32);
    reset->setMinimumHeight(32);
    connect(start, &QPushButton::clicked, this, [this] { command("start " + std::to_string(laps_->value())); });
    connect(reset, &QPushButton::clicked, this, [this] { command("reset"); });
    auto * top = new QHBoxLayout;
    top->addWidget(laps_);
    top->addWidget(start, 1);
    top->addWidget(reset);

    status_ = new QLabel;
    status_->setStyleSheet("font-size:15pt; font-weight:bold;");
    auto * grid = new QGridLayout;
    const QStringList head{"P", "car", "lap", "last", "best", "total"};
    for (int c = 0; c < head.size(); ++c) {
      grid->addWidget(new QLabel("<b>" + head[c] + "</b>"), 0, c);
    }
    for (int r = 0; r < static_cast<int>(kCars.size()); ++r) {
      for (int c = 0; c < head.size(); ++c) {
        cells_[r][c] = new QLabel;
        grid->addWidget(cells_[r][c], r + 1, c);
      }
    }
    auto * layout = new QVBoxLayout;
    layout->addLayout(top);
    layout->addWidget(status_);
    auto * hint = new QLabel("Set Start: nuc1 / nuc4 / nuc5 in the toolbar. Only cars with a line join.");
    hint->setWordWrap(true);
    layout->addWidget(hint);
    auto * clear = new QPushButton("Clear start lines");
    clear->setToolTip("RESET first. Removes all saved start lines so you can choose new participants.");
    connect(clear, &QPushButton::clicked, this, [this] { command("clear_lines"); });
    layout->addWidget(clear);
    layout->addLayout(grid);
    layout->addStretch();
    setLayout(layout);

    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { refresh(); });
    timer->start(100);
  }

  void onInitialize() override
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    cmd_pub_ = node_->create_publisher<std_msgs::msg::String>("/race/command", 10);
    state_.subscribe(node_);
  }

  void save(rviz_common::Config config) const override
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Laps", laps_->value());
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    int v;
    if (config.mapGetInt("Laps", &v)) {
      laps_->setValue(v);
    }
  }

private:
  void command(const std::string & text)
  {
    if (!cmd_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    cmd_pub_->publish(msg);
  }

  void refresh()
  {
    QJsonObject s;
    double age;
    if (!state_.latest(&s, &age)) {
      status_->setText("<font color='#c62828'>no lap_timer</font>");
      return;
    }
    const QString phase = s["phase"].toString();
    QString text;
    if (phase == "countdown") {
      text = QString("GO in %1").arg(s["countdown"].toDouble(), 0, 'f', 1);
    } else if (phase == "running") {
      text = "RUNNING  " + fmtTime(s["clock"].toDouble() + age);
    } else if (phase == "finished") {
      text = "FINISHED";
    } else {
      text = "ready (" + s["start_mode"].toString("shared") + ")";
    }
    if (!s["line_set"].toBool()) {
      text += "  <font color='#c62828'>(set the start line)</font>";
    }
    status_->setText(text);

    // rows sorted by race position; cars not in the race below
    const QJsonObject cars = s["cars"].toObject();
    std::vector<std::pair<int, QString>> order;
    for (const auto & [car, col] : kCars) {
      const QJsonObject c = cars[car].toObject();
      order.emplace_back(c["in_race"].toBool() ? c["pos"].toInt() : 99, car);
    }
    std::sort(order.begin(), order.end());
    for (int r = 0; r < static_cast<int>(order.size()); ++r) {
      const QString & car = order[r].second;
      const QJsonObject c = cars[car].toObject();
      const bool in = c["in_race"].toBool();
      QStringList v{in ? QString::number(c["pos"].toInt()) : "",
                    QString("<b><font color='%1'>%2</font></b>").arg(carColor(car), car),
                    in ? QString("%1/%2").arg(c["lap"].toInt()).arg(c["laps_total"].toInt())
                       : (!c["line_set"].toBool() ? "set start line" : (c["on_track"].toBool() ? "ready" : "off")),
                    in ? fmtTime(c["last"]) : "", in ? fmtTime(c["best"]) : "",
                    in && c["finished"].toBool() ? fmtTime(c["total"]) : ""};
      for (int k = 0; k < v.size(); ++k) {
        cells_[r][k]->setText(v[k]);
      }
    }
  }

  QSpinBox * laps_;
  QLabel * status_;
  QLabel * cells_[3][6];
  RaceStateSub state_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
};

// ---------------------------------------------------------------- driver HUD (one car)

class RaceHud : public rviz_common::Panel
{
public:
  explicit RaceHud(QWidget * parent = nullptr) : rviz_common::Panel(parent)
  {
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(17, 17, 17));
    pal.setColor(QPalette::WindowText, Qt::white);
    setPalette(pal);
    pos_ = new QLabel;
    lap_ = new QLabel;
    big_ = new QLabel;
    small_ = new QLabel;
    pos_->setStyleSheet("font-size:44pt; font-weight:bold; color:white; background:transparent;");
    lap_->setStyleSheet("font-size:32pt; font-weight:bold; color:white; background:transparent;");
    big_->setStyleSheet("font-size:56pt; font-weight:bold; font-family:monospace; color:white; background:transparent;");
    small_->setStyleSheet("font-size:20pt; font-family:monospace; color:#ccc; background:transparent;");
    lap_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    big_->setAlignment(Qt::AlignCenter);
    small_->setAlignment(Qt::AlignCenter);
    auto * top = new QHBoxLayout;
    top->addWidget(pos_);
    top->addStretch();
    top->addWidget(lap_);
    auto * layout = new QVBoxLayout;
    layout->addLayout(top);
    layout->addWidget(big_);
    layout->addWidget(small_);
    setLayout(layout);

    auto * timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] { refresh(); });
    timer->start(33);
  }

  void onInitialize() override
  {
    state_.subscribe(getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
  }

  void save(rviz_common::Config config) const override
  {
    rviz_common::Panel::save(config);
    config.mapSetValue("Car", car_);
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    QString car;
    if (config.mapGetString("Car", &car)) {
      car_ = car;
    }
  }

private:
  void show(const QString & pos, const QString & lap, const QString & big, const QString & small)
  {
    pos_->setText(pos);
    lap_->setText(lap);
    big_->setText(big);
    small_->setText(small);
  }

  void refresh()
  {
    const QString name = QString("<font color='%1'>%2</font>").arg(carColor(car_), car_);
    QJsonObject s;
    double age;
    if (!state_.latest(&s, &age)) {
      show(name, "", "", "no race control");
      return;
    }
    const QString phase = s["phase"].toString();
    const QJsonObject c = s["cars"].toObject()[car_].toObject();
    if (phase == "idle") {
      show(name, "", "READY", "waiting for the start");
      return;
    }
    if (phase == "countdown") {
      const double t = s["countdown"].toDouble() - age;
      show(name, QString("%1 LAPS").arg(s["laps_total"].toInt()),
           QString("<font color='#ffd700'>%1</font>").arg(t > 0 ? QString::number(static_cast<int>(t) + 1) : "GO!"),
           "");
      return;
    }
    if (!c["in_race"].toBool()) {
      show(name, "", "", "not entered (needs start line and live pose at GO)");
      return;
    }
    const QString pos = QString("P%1<span style='font-size:24pt; color:#888;'>/%2</span>")
      .arg(c["pos"].toInt()).arg(s["n_in_race"].toInt());
    const int total = c["laps_total"].toInt();
    const QString times = QString("LAST %1   BEST %2").arg(fmtTime(c["last"]), fmtTime(c["best"]));
    if (c["finished"].toBool()) {
      show(pos, "FINISH", "<font color='#ffd700'>" + fmtTime(c["total"]) + "</font>", times);
      return;
    }
    const double clock = s["clock"].toDouble() + age;
    const QString lap = QString("LAP %1/%2").arg(std::min(c["lap"].toInt() + 1, total)).arg(total);
    const QString big = clock < 1.5 ? "<font color='#4bff4b'>GO!</font>"
                                    : fmtTime(c["lap_time"].toDouble() + age);
    show(pos, lap, big, times);
  }

  QString car_{"nuc1"};
  QLabel * pos_;
  QLabel * lap_;
  QLabel * big_;
  QLabel * small_;
  RaceStateSub state_;
};

}  // namespace fleet_rviz_tools

PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::StartLineTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::RacePanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::RaceHud, rviz_common::Panel)

PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::StartLineNuc1, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::StartLineNuc4, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::StartLineNuc5, rviz_common::Tool)
