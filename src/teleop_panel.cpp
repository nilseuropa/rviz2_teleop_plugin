#include "rviz2_teleop_plugin/teleop_panel.hpp"

#include <algorithm>

#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace rviz2_teleop_plugin
{

TeleopPanel::TeleopPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setFocusPolicy(Qt::StrongFocus);

  auto * layout = new QVBoxLayout();
  layout->setContentsMargins(6, 6, 6, 6);

  auto * topic_row = new QHBoxLayout();
  topic_row->addWidget(new QLabel("Topic"));
  topic_edit_ = new QLineEdit(QString::fromStdString(topic_));
  topic_row->addWidget(topic_edit_);
  layout->addLayout(topic_row);

  auto * linear_row = new QHBoxLayout();
  linear_row->addWidget(new QLabel("Max linear"));
  max_linear_spin_ = new QDoubleSpinBox();
  max_linear_spin_->setRange(0.0, 100.0);
  max_linear_spin_->setDecimals(3);
  max_linear_spin_->setSingleStep(0.05);
  max_linear_spin_->setValue(max_linear_);
  linear_row->addWidget(max_linear_spin_);
  layout->addLayout(linear_row);

  auto * angular_row = new QHBoxLayout();
  angular_row->addWidget(new QLabel("Max angular"));
  max_angular_spin_ = new QDoubleSpinBox();
  max_angular_spin_->setRange(0.0, 100.0);
  max_angular_spin_->setDecimals(3);
  max_angular_spin_->setSingleStep(0.05);
  max_angular_spin_->setValue(max_angular_);
  angular_row->addWidget(max_angular_spin_);
  layout->addLayout(angular_row);

  auto * rate_row = new QHBoxLayout();
  rate_row->addWidget(new QLabel("Publish rate (Hz)"));
  publish_rate_spin_ = new QDoubleSpinBox();
  publish_rate_spin_->setRange(1.0, 100.0);
  publish_rate_spin_->setDecimals(1);
  publish_rate_spin_->setSingleStep(1.0);
  publish_rate_spin_->setValue(publish_rate_hz_);
  rate_row->addWidget(publish_rate_spin_);
  layout->addLayout(rate_row);

  status_label_ = new QLabel("Click here and use arrow keys");
  layout->addWidget(status_label_);

  setLayout(layout);

  connect(topic_edit_, &QLineEdit::editingFinished, this, [this]() {
    const auto trimmed = topic_edit_->text().trimmed();
    topic_ = trimmed.isEmpty() ? std::string("cmd_vel") : trimmed.toStdString();
    topic_edit_->setText(QString::fromStdString(topic_));
    updatePublisher();
  });

  connect(max_linear_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    max_linear_ = value;
    updateFromKeys();
  });

  connect(max_angular_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    max_angular_ = value;
    updateFromKeys();
  });

  connect(publish_rate_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    publish_rate_hz_ = value;
    updatePublishRate();
  });

  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &TeleopPanel::publishCurrent);
  updatePublishRate();
}

void TeleopPanel::onInitialize()
{
  auto ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (ros_node) {
    node_ = ros_node->get_raw_node();
  }
  updatePublisher();
}

void TeleopPanel::keyPressEvent(QKeyEvent * event)
{
  if (!event || event->isAutoRepeat()) {
    return;
  }

  switch (event->key()) {
    case Qt::Key_Up:
      up_pressed_ = true;
      break;
    case Qt::Key_Down:
      down_pressed_ = true;
      break;
    case Qt::Key_Left:
      left_pressed_ = true;
      break;
    case Qt::Key_Right:
      right_pressed_ = true;
      break;
    default:
      rviz_common::Panel::keyPressEvent(event);
      return;
  }

  updateFromKeys();
}

void TeleopPanel::keyReleaseEvent(QKeyEvent * event)
{
  if (!event || event->isAutoRepeat()) {
    return;
  }

  switch (event->key()) {
    case Qt::Key_Up:
      up_pressed_ = false;
      break;
    case Qt::Key_Down:
      down_pressed_ = false;
      break;
    case Qt::Key_Left:
      left_pressed_ = false;
      break;
    case Qt::Key_Right:
      right_pressed_ = false;
      break;
    default:
      rviz_common::Panel::keyReleaseEvent(event);
      return;
  }

  updateFromKeys();
}

void TeleopPanel::mousePressEvent(QMouseEvent * event)
{
  setFocus(Qt::MouseFocusReason);
  rviz_common::Panel::mousePressEvent(event);
}

void TeleopPanel::publishCurrent()
{
  if (!publisher_) {
    return;
  }

  publisher_->publish(last_msg_);
}

void TeleopPanel::updatePublisher()
{
  if (!node_) {
    return;
  }

  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_, rclcpp::QoS(10));
}

void TeleopPanel::updateFromKeys()
{
  const int linear_dir = (up_pressed_ ? 1 : 0) + (down_pressed_ ? -1 : 0);
  const int angular_dir = (left_pressed_ ? 1 : 0) + (right_pressed_ ? -1 : 0);

  last_msg_ = geometry_msgs::msg::Twist();
  last_msg_.linear.x = static_cast<double>(linear_dir) * max_linear_;
  last_msg_.angular.z = static_cast<double>(angular_dir) * max_angular_;

  updateStatusLabel();
}

void TeleopPanel::updatePublishRate()
{
  if (!publish_timer_) {
    return;
  }

  const double clamped = std::max(1.0, publish_rate_hz_);
  const int interval_ms = static_cast<int>(1000.0 / clamped);
  publish_timer_->setInterval(interval_ms);
  publish_timer_->start();
}

void TeleopPanel::updateStatusLabel()
{
  if (!status_label_) {
    return;
  }

  status_label_->setText(QString("lin.x: %1  ang.z: %2")
    .arg(last_msg_.linear.x, 0, 'f', 2)
    .arg(last_msg_.angular.z, 0, 'f', 2));
}

}  // namespace rviz2_teleop_plugin

PLUGINLIB_EXPORT_CLASS(rviz2_teleop_plugin::TeleopPanel, rviz_common::Panel)
