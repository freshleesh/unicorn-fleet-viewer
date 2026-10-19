// Fleet autodrive panel: per-car ON/OFF toggles + ALL ON / ALL OFF, and a per-car speed cap slider.
// Speed cap (1..8 m/s) -> /<car>/max_speed (latched Float32): the ring planner's speed and the
// joystick full-stick speed in simple_mux.
// Uses the simple_mux joystick protocol, so the racing stack is untouched:
//   ON  = Joy with RB (button 5) pressed            -> mux hands over to the planner (autodrive)
//   OFF = Joy with LB (button 4) pressed, sticks 0  -> mux switches to human drive at speed 0
// Published on /<car>/joy (shared domain); fleet_relay forwards it to the car's /joy.
#include <map>

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/panel.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rviz_common/config.hpp"

namespace fleet_rviz_tools
{

class AutodrivePanel : public rviz_common::Panel
{
public:
  explicit AutodrivePanel(QWidget * parent = nullptr) : rviz_common::Panel(parent)
  {
    auto * grid = new QGridLayout;
    int row = 0;
    for (const auto & [car, color] : cars_) {
      auto * label = new QLabel(QString("<b><font color='%1'>%2</font></b>").arg(color, car));
      auto * btn = new QPushButton("OFF");
      btn->setCheckable(true);
      btn->setMinimumHeight(32);
      connect(btn, &QPushButton::toggled, this, [this, car = car, btn](bool on) {
        btn->setText(on ? "AUTO" : "OFF");
        btn->setStyleSheet(on ? "background-color:#2e7d32; color:white; font-weight:bold;" : "");
        send(car, on);
      });
      auto * slider = new QSlider(Qt::Horizontal);
      slider->setRange(10, 80);                         // 0.1 m/s steps, 1.0 .. 8.0
      slider->setValue(20);
      auto * value = new QLabel("2.0 m/s");
      value->setMinimumWidth(56);
      connect(slider, &QSlider::valueChanged, this, [this, car = car, value](int v) {
        value->setText(QString::number(v / 10.0, 'f', 1) + " m/s");
        sendCap(car, v / 10.0);
      });
      grid->addWidget(label, row, 0);
      grid->addWidget(btn, row, 1);
      grid->addWidget(slider, row, 2);
      grid->addWidget(value, row, 3);
      buttons_[car] = btn;
      sliders_[car] = slider;
      ++row;
    }
    auto * all_on = new QPushButton("ALL AUTO");
    auto * all_off = new QPushButton("ALL OFF");
    all_off->setStyleSheet("background-color:#c62828; color:white; font-weight:bold;");
    all_on->setMinimumHeight(32);
    all_off->setMinimumHeight(40);
    connect(all_on, &QPushButton::clicked, this, [this] { setAll(true); });
    connect(all_off, &QPushButton::clicked, this, [this] { setAll(false); });
    grid->addWidget(all_on, row, 0, 1, 4);
    grid->addWidget(all_off, row + 1, 0, 1, 4);
    grid->setColumnStretch(2, 1);
    auto * layout = new QVBoxLayout;
    layout->addLayout(grid);
    layout->addStretch();
    setLayout(layout);
  }

  void onInitialize() override
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    for (const auto & [car, color] : cars_) {
      pubs_[car] = node_->create_publisher<sensor_msgs::msg::Joy>("/" + car.toStdString() + "/joy", 10);
      cap_pubs_[car] = node_->create_publisher<std_msgs::msg::Float32>(
        "/" + car.toStdString() + "/max_speed", rclcpp::QoS(1).transient_local());
      sendCap(car, sliders_[car]->value() / 10.0);     // cars get the cap as soon as the panel is up
    }
  }

  void save(rviz_common::Config config) const override
  {
    rviz_common::Panel::save(config);
    for (const auto & [car, slider] : sliders_) {
      config.mapSetValue("MaxSpeed_" + car, slider->value() / 10.0);
    }
  }

  void load(const rviz_common::Config & config) override
  {
    rviz_common::Panel::load(config);
    for (auto & [car, slider] : sliders_) {
      float v;
      if (config.mapGetFloat("MaxSpeed_" + car, &v)) {
        slider->setValue(static_cast<int>(v * 10.0f + 0.5f));
      }
    }
  }

private:
  void setAll(bool on)
  {
    for (auto & [car, btn] : buttons_) {
      if (btn->isChecked() == on) {
        send(car, on);            // re-send anyway: the car may have been switched from its pad
      } else {
        btn->setChecked(on);      // toggled() sends
      }
    }
  }

  void sendCap(const QString & car, double v)
  {
    auto it = cap_pubs_.find(car);
    if (it == cap_pubs_.end()) {
      return;
    }
    std_msgs::msg::Float32 msg;
    msg.data = static_cast<float>(v);
    it->second->publish(msg);
  }

  void send(const QString & car, bool on)
  {
    auto it = pubs_.find(car);
    if (it == pubs_.end()) {
      return;
    }
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = node_->now();
    msg.axes.assign(8, 0.0f);
    msg.buttons.assign(12, 0);
    msg.buttons[on ? 5 : 4] = 1;
    for (int i = 0; i < 3; ++i) {  // a few copies: WiFi may drop one
      it->second->publish(msg);
    }
  }

  const std::vector<std::pair<QString, QString>> cars_{
    {"nuc1", "#ff6419"}, {"nuc4", "#3296ff"}, {"nuc5", "#4bff4b"}};
  std::map<QString, QPushButton *> buttons_;
  std::map<QString, QSlider *> sliders_;
  std::map<QString, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> cap_pubs_;
  std::map<QString, rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> pubs_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace fleet_rviz_tools

PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::AutodrivePanel, rviz_common::Panel)
