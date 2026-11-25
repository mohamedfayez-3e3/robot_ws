
#ifndef GRIPPER_RVIZ_PANEL__GRIPPER_PANEL_HPP_
#define GRIPPER_RVIZ_PANEL__GRIPPER_PANEL_HPP_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class QSlider;
class QLabel;
class QPushButton;

namespace gripper_rviz_panel
{

class GripperPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GripperPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private:
  void publishValue(double value);
  void updateSlider(int value);

  QSlider * slider_;
  QLabel * value_label_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
};

}  // namespace gripper_rviz_panel

#endif
