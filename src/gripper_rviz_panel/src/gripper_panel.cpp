
#include "gripper_rviz_panel/gripper_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QPushButton>

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace gripper_rviz_panel
{

GripperPanel::GripperPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * main_layout = new QVBoxLayout;

  auto * title = new QLabel("Direct Gripper Control");
  title->setAlignment(Qt::AlignCenter);

  auto * note = new QLabel("Direct command; bypasses MoveIt Plan / Execute.");
  note->setWordWrap(true);

  value_label_ = new QLabel("Gripper: 0.00");
  value_label_->setAlignment(Qt::AlignCenter);

  slider_ = new QSlider(Qt::Horizontal);
  slider_->setRange(0, 100);
  slider_->setValue(0);

  auto * buttons_layout = new QHBoxLayout;

  auto * open_button = new QPushButton("Open");
  auto * close_button = new QPushButton("Close");

  buttons_layout->addWidget(open_button);
  buttons_layout->addWidget(close_button);

  main_layout->addWidget(title);
  main_layout->addWidget(note);
  main_layout->addWidget(value_label_);
  main_layout->addWidget(slider_);
  main_layout->addLayout(buttons_layout);

  setLayout(main_layout);

  connect(
    slider_, &QSlider::valueChanged,
    this, &GripperPanel::updateSlider);

  connect(open_button, &QPushButton::clicked, this, [this]() {
    slider_->setValue(0);
  });

  connect(close_button, &QPushButton::clicked, this, [this]() {
    slider_->setValue(100);
  });
}

void GripperPanel::onInitialize()
{
  auto node = getDisplayContext()
    ->getRosNodeAbstraction()
    .lock()
    ->get_raw_node();

  publisher_ =
    node->create_publisher<std_msgs::msg::Float64>(
      "/gripper_command", 10);
}

void GripperPanel::updateSlider(int value)
{
  double normalized_value = value / 100.0;

  value_label_->setText(
    QString("Gripper: %1").arg(normalized_value, 0, 'f', 2));

  publishValue(normalized_value);
}

void GripperPanel::publishValue(double value)
{
  if (!publisher_) {
    return;
  }

  std_msgs::msg::Float64 msg;
  msg.data = value;

  publisher_->publish(msg);
}

}  // namespace gripper_rviz_panel

PLUGINLIB_EXPORT_CLASS(
  gripper_rviz_panel::GripperPanel,
  rviz_common::Panel)
