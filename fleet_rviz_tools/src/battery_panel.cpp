#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <QGridLayout>
#include <QLabel>
#include <QProgressBar>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "battery_estimate.hpp"

namespace fleet_rviz_tools {
class BatteryPanel : public rviz_common::Panel {
  using Clock = std::chrono::steady_clock;
  struct Sample {
    std::mutex mutex;
    double voltage = NAN;
    Clock::time_point received{};
    bool seen = false;
  };
  struct Row {
    QString car;
    QLabel *voltage;
    QProgressBar *percent;
    QSpinBox *cells;
    std::shared_ptr<Sample> sample = std::make_shared<Sample>();
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr sub;
  };
public:
  explicit BatteryPanel(QWidget *parent = nullptr) : rviz_common::Panel(parent) {
    auto *layout = new QVBoxLayout;
    auto *note = new QLabel(QString::fromUtf8("배터리 잔량 ≈ 전압 기반 추정 · 기본 4S LiPo"));
    note->setWordWrap(true);
    note->setToolTip(QString::fromUtf8("셀당 3.50~4.20V를 0~100%로 선형 환산합니다. 실제 잔량은 배터리 상태와 주행 부하에 따라 다릅니다."));
    layout->addWidget(note);
    auto *grid = new QGridLayout;
    int i = 0;
    for (const QString car : {QString("nuc1"), QString("nuc4"), QString("nuc5")}) {
      Row row;
      row.car = car;
      row.voltage = new QLabel(QString::fromUtf8("수신 대기"));
      row.percent = new QProgressBar;
      row.percent->setRange(0, 100);
      row.percent->setValue(0);
      row.percent->setFormat("—");
      row.cells = new QSpinBox;
      row.cells->setRange(1, 12);
      row.cells->setValue(4);
      row.cells->setSuffix("S");
      row.cells->setToolTip(QString::fromUtf8("배터리 직렬 셀 수"));
      grid->addWidget(new QLabel(car), i, 0);
      grid->addWidget(row.cells, i, 1);
      grid->addWidget(row.voltage, i, 2);
      grid->addWidget(row.percent, i++, 3);
      rows_.push_back(std::move(row));
    }
    grid->setColumnStretch(3, 1);
    layout->addLayout(grid);
    layout->addStretch();
    setLayout(layout);
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, [this] { refresh(); });
    timer_->start(500);
  }

  void onInitialize() override {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    for (auto &row : rows_) {
      auto sample = row.sample;
      row.sub = node_->create_subscription<vesc_msgs::msg::VescStateStamped>(
        "/" + row.car.toStdString() + "/vesc/core", rclcpp::QoS(1).best_effort(),
        [sample](vesc_msgs::msg::VescStateStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> guard(sample->mutex);
          sample->voltage = msg->state.voltage_input;
          sample->received = Clock::now();
          sample->seen = true;
        });
    }
  }

  void save(rviz_common::Config config) const override {
    rviz_common::Panel::save(config);
    for (const auto &row : rows_) config.mapSetValue("Cells_" + row.car, row.cells->value());
  }
  void load(const rviz_common::Config &config) override {
    rviz_common::Panel::load(config);
    for (auto &row : rows_) {
      int cells;
      if (config.mapGetInt("Cells_" + row.car, &cells)) row.cells->setValue(cells);
    }
  }
private:
  void refresh() {
    for (auto &row : rows_) {
      std::lock_guard<std::mutex> guard(row.sample->mutex);
      const auto &s = *row.sample;
      const bool stale = s.seen && Clock::now() - s.received > std::chrono::seconds(3);
      double pct = voltagePercent(s.voltage, row.cells->value());
      if (!s.seen || stale || !std::isfinite(pct)) {
        row.voltage->setText(!s.seen ? QString::fromUtf8("수신 대기") :
          stale ? QString::fromUtf8("수신 끊김") : QString::fromUtf8("전압 오류"));
        row.percent->setValue(0);
        row.percent->setFormat("—");
        row.percent->setStyleSheet("");
        continue;
      }
      const double cell = s.voltage / row.cells->value();
      const char *color = cell < 3.65 ? "#c62828" : cell < 3.75 ? "#ef9b0f" : "#2e7d32";
      row.voltage->setText(QString::number(s.voltage, 'f', 1) + " V");
      row.percent->setValue(static_cast<int>(std::round(pct)));
      row.percent->setFormat(QString::fromUtf8("≈ %p%"));
      row.percent->setStyleSheet(QString("QProgressBar::chunk { background: %1; }").arg(color));
    }
  }
  std::vector<Row> rows_;
  QTimer *timer_;
  rclcpp::Node::SharedPtr node_;
};
}  // namespace fleet_rviz_tools
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::BatteryPanel, rviz_common::Panel)
