// "2D Pose Estimate" per car: same tool as RViz's, but with the car's name, colour and topic.
#include <QColor>
#include <QPainter>
#include <QPixmap>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_default_plugins/tools/pose_estimate/initial_pose_tool.hpp"
#include "rviz_rendering/objects/arrow.hpp"

namespace fleet_rviz_tools
{

class CarPoseTool : public rviz_default_plugins::tools::InitialPoseTool
{
public:
  CarPoseTool(const char * car, QColor color) : car_(car), color_(color) {}

  void onInitialize() override
  {
    InitialPoseTool::onInitialize();
    setName(QString("Pose: %1").arg(car_));
    getPropertyContainer()->subProp("Topic")->setValue(QString("/%1/initialpose").arg(car_));
    arrow_->setColor(color_.redF(), color_.greenF(), color_.blueF(), 1.0f);

    QPixmap icon = rviz_common::loadPixmap("package://rviz_default_plugins/icons/classes/SetInitialPose.png");
    QPainter p(&icon);                         // recolour the green arrow, keep its alpha
    p.setCompositionMode(QPainter::CompositionMode_SourceIn);
    p.fillRect(icon.rect(), color_);
    p.end();
    setIcon(QIcon(icon));
  }

private:
  QString car_;
  QColor color_;
};

// colours match the car markers in fleet_relay (fleet_env.sh FLEET_COLOR)
struct PoseNuc1 : CarPoseTool { PoseNuc1() : CarPoseTool("nuc1", QColor(255, 100, 25)) {} };
struct PoseNuc4 : CarPoseTool { PoseNuc4() : CarPoseTool("nuc4", QColor(50, 150, 255)) {} };
struct PoseNuc5 : CarPoseTool { PoseNuc5() : CarPoseTool("nuc5", QColor(75, 255, 75)) {} };

}  // namespace fleet_rviz_tools

PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::PoseNuc1, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::PoseNuc4, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(fleet_rviz_tools::PoseNuc5, rviz_common::Tool)
