#ifndef RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_
#define RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_

#include <memory>
#include <string>

#include <QWidget>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QDoubleSpinBox;
class QLineEdit;
class QKeyEvent;
class QMouseEvent;
class QLabel;
class QTimer;

namespace rviz2_teleop_plugin
{

class TeleopPanel : public rviz_common::Panel
{
public:
  explicit TeleopPanel(QWidget * parent = nullptr);

  void onInitialize() override;

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;

private:
  void updatePublisher();
  void updateFromKeys();
  void publishCurrent();
  void updatePublishRate();
  void updateStatusLabel();

  QLineEdit * topic_edit_{};
  QDoubleSpinBox * max_linear_spin_{};
  QDoubleSpinBox * max_angular_spin_{};
  QDoubleSpinBox * publish_rate_spin_{};
  QLabel * status_label_{};
  QTimer * publish_timer_{};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  std::string topic_ = "cmd_vel";
  double max_linear_ = 0.5;
  double max_angular_ = 1.0;
  double publish_rate_hz_ = 10.0;

  bool up_pressed_ = false;
  bool down_pressed_ = false;
  bool left_pressed_ = false;
  bool right_pressed_ = false;

  geometry_msgs::msg::Twist last_msg_{};
};

}  // namespace rviz2_teleop_plugin

#endif  // RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_
