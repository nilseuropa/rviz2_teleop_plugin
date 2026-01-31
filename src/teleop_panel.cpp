#include "rviz2_teleop_plugin/teleop_panel.hpp"

#include <algorithm>

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace rviz2_teleop_plugin
{

TeleopPanelBase::TeleopPanelBase(QWidget * parent)
: rviz_common::Panel(parent)
{
  main_layout_ = new QVBoxLayout();
  main_layout_->setContentsMargins(6, 6, 6, 6);

  auto * topic_row = new QHBoxLayout();
  topic_row->addWidget(new QLabel("Topic"));
  topic_edit_ = new QLineEdit(QString::fromStdString(topic_));
  topic_row->addWidget(topic_edit_);
  main_layout_->addLayout(topic_row);

  auto * linear_row = new QHBoxLayout();
  linear_row->addWidget(new QLabel("Max linear"));
  max_linear_spin_ = new QDoubleSpinBox();
  max_linear_spin_->setRange(0.0, 100.0);
  max_linear_spin_->setDecimals(3);
  max_linear_spin_->setSingleStep(0.05);
  max_linear_spin_->setValue(max_linear_);
  linear_row->addWidget(max_linear_spin_);
  main_layout_->addLayout(linear_row);

  auto * angular_row = new QHBoxLayout();
  angular_row->addWidget(new QLabel("Max angular"));
  max_angular_spin_ = new QDoubleSpinBox();
  max_angular_spin_->setRange(0.0, 100.0);
  max_angular_spin_->setDecimals(3);
  max_angular_spin_->setSingleStep(0.05);
  max_angular_spin_->setValue(max_angular_);
  angular_row->addWidget(max_angular_spin_);
  main_layout_->addLayout(angular_row);

  auto * rate_row = new QHBoxLayout();
  rate_row->addWidget(new QLabel("Publish rate (Hz)"));
  publish_rate_spin_ = new QDoubleSpinBox();
  publish_rate_spin_->setRange(1.0, 100.0);
  publish_rate_spin_->setDecimals(1);
  publish_rate_spin_->setSingleStep(1.0);
  publish_rate_spin_->setValue(publish_rate_hz_);
  rate_row->addWidget(publish_rate_spin_);
  main_layout_->addLayout(rate_row);

  status_label_ = new QLabel("Click here and use arrow keys");
  main_layout_->addWidget(status_label_);

  setLayout(main_layout_);

  connect(topic_edit_, &QLineEdit::editingFinished, this, [this]() {
    const auto trimmed = topic_edit_->text().trimmed();
    topic_ = trimmed.isEmpty() ? std::string("cmd_vel") : trimmed.toStdString();
    topic_edit_->setText(QString::fromStdString(topic_));
    updatePublisher();
  });

  connect(max_linear_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    max_linear_ = value;
    refreshCommandFromInput();
  });

  connect(max_angular_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    max_angular_ = value;
    refreshCommandFromInput();
  });

  connect(publish_rate_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    publish_rate_hz_ = value;
    updatePublishRate();
  });

  publish_timer_ = new QTimer(this);
  connect(publish_timer_, &QTimer::timeout, this, &TeleopPanelBase::publishCurrent);
  updatePublishRate();
}

void TeleopPanelBase::onInitialize()
{
  auto ros_node = getDisplayContext()->getRosNodeAbstraction().lock();
  if (ros_node) {
    node_ = ros_node->get_raw_node();
  }
  updatePublisher();
}

void TeleopPanelBase::publishCurrent()
{
  if (!publisher_) {
    return;
  }

  publisher_->publish(last_msg_);
}

void TeleopPanelBase::updatePublisher()
{
  if (!node_) {
    return;
  }

  publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(topic_, rclcpp::QoS(10));
}

void TeleopPanelBase::updatePublishRate()
{
  if (!publish_timer_) {
    return;
  }

  const double clamped = std::max(1.0, publish_rate_hz_);
  const int interval_ms = static_cast<int>(1000.0 / clamped);
  publish_timer_->setInterval(interval_ms);
  publish_timer_->start();
}

void TeleopPanelBase::setCommand(double linear, double angular)
{
  last_msg_ = geometry_msgs::msg::Twist();
  last_msg_.linear.x = linear;
  last_msg_.angular.z = angular;
  updateStatusLabel();
}

void TeleopPanelBase::updateStatusLabel()
{
  if (!status_label_) {
    return;
  }

  status_label_->setText(QString("lin.x: %1  ang.z: %2")
    .arg(last_msg_.linear.x, 0, 'f', 2)
    .arg(last_msg_.angular.z, 0, 'f', 2));
}

KeyTeleopPanel::KeyTeleopPanel(QWidget * parent)
: TeleopPanelBase(parent)
{
  setFocusPolicy(Qt::StrongFocus);
  status_label_->setText("Click here and use arrow keys");
}

void KeyTeleopPanel::keyPressEvent(QKeyEvent * event)
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

  refreshCommandFromInput();
}

void KeyTeleopPanel::keyReleaseEvent(QKeyEvent * event)
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

  refreshCommandFromInput();
}

void KeyTeleopPanel::mousePressEvent(QMouseEvent * event)
{
  setFocus(Qt::MouseFocusReason);
  rviz_common::Panel::mousePressEvent(event);
}

void KeyTeleopPanel::refreshCommandFromInput()
{
  const int linear_dir = (up_pressed_ ? 1 : 0) + (down_pressed_ ? -1 : 0);
  const int angular_dir = (left_pressed_ ? 1 : 0) + (right_pressed_ ? -1 : 0);

  setCommand(static_cast<double>(linear_dir) * max_linear_,
    static_cast<double>(angular_dir) * max_angular_);
}

PadTeleopPanel::PadTeleopPanel(QWidget * parent)
: TeleopPanelBase(parent)
{
  auto * grid = new QGridLayout();
  grid->setSpacing(4);

  addPadButton(grid, 0, 0, "↖", 1.0, 1.0);
  addPadButton(grid, 0, 1, "↑", 1.0, 0.0);
  addPadButton(grid, 0, 2, "↗", 1.0, -1.0);
  addPadButton(grid, 1, 0, "←", 0.0, 1.0);
  addPadButton(grid, 1, 2, "→", 0.0, -1.0);
  addPadButton(grid, 2, 0, "↙", -1.0, 1.0);
  addPadButton(grid, 2, 1, "↓", -1.0, 0.0);
  addPadButton(grid, 2, 2, "↘", -1.0, -1.0);

  auto * pad_widget = new QWidget();
  pad_widget->setLayout(grid);
  main_layout_->addWidget(pad_widget);

  main_layout_->removeWidget(status_label_);
  main_layout_->addWidget(status_label_);
  status_label_->setText("Click a pad button to drive");
}

void PadTeleopPanel::addPadButton(QGridLayout * grid, int row, int col, const QString & text,
  double linear, double angular)
{
  auto * button = new QPushButton(text);
  button->setMinimumSize(36, 36);
  button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  grid->addWidget(button, row, col);

  connect(button, &QPushButton::pressed, this, [this, button, linear, angular]() {
    active_button_ = button;
    linear_dir_ = static_cast<int>(linear);
    angular_dir_ = static_cast<int>(angular);
    refreshCommandFromInput();
  });

  connect(button, &QPushButton::released, this, [this, button]() {
    clearIfActive(button);
  });
}

void PadTeleopPanel::clearIfActive(QPushButton * button)
{
  if (button != active_button_) {
    return;
  }

  active_button_ = nullptr;
  linear_dir_ = 0;
  angular_dir_ = 0;
  refreshCommandFromInput();
}

void PadTeleopPanel::refreshCommandFromInput()
{
  if (!active_button_) {
    setCommand(0.0, 0.0);
    return;
  }

  setCommand(static_cast<double>(linear_dir_) * max_linear_,
    static_cast<double>(angular_dir_) * max_angular_);
}

}  // namespace rviz2_teleop_plugin

PLUGINLIB_EXPORT_CLASS(rviz2_teleop_plugin::KeyTeleopPanel, rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(rviz2_teleop_plugin::PadTeleopPanel, rviz_common::Panel)
