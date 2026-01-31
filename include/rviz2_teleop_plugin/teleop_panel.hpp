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
class QPushButton;
class QVBoxLayout;
class QGridLayout;

namespace rviz2_teleop_plugin
{

class TeleopPanelBase : public rviz_common::Panel
{
public:
  explicit TeleopPanelBase(QWidget * parent = nullptr);

  void onInitialize() override;

protected:
  void updatePublisher();
  void publishCurrent();
  void updatePublishRate();
  void updateStatusLabel();
  void setCommand(double linear, double angular);
  virtual void refreshCommandFromInput() = 0;

  QVBoxLayout * main_layout_{};
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

  geometry_msgs::msg::Twist last_msg_{};
};

class KeyTeleopPanel : public TeleopPanelBase
{
public:
  explicit KeyTeleopPanel(QWidget * parent = nullptr);

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;

private:
  void refreshCommandFromInput() override;

  bool up_pressed_ = false;
  bool down_pressed_ = false;
  bool left_pressed_ = false;
  bool right_pressed_ = false;
};

class PadTeleopPanel : public TeleopPanelBase
{
public:
  explicit PadTeleopPanel(QWidget * parent = nullptr);

private:
  void addPadButton(QGridLayout * grid, int row, int col, const QString & text,
    double linear, double angular);
  void clearIfActive(QPushButton * button);
  void refreshCommandFromInput() override;

  QPushButton * active_button_{};
  int linear_dir_ = 0;
  int angular_dir_ = 0;
};

}  // namespace rviz2_teleop_plugin

#endif  // RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_
