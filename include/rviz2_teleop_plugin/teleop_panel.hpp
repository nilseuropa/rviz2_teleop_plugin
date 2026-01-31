#ifndef RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_
#define RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_

#include <memory>
#include <string>

#include <QWidget>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/joy.hpp>

class QDoubleSpinBox;
class QLineEdit;
class QKeyEvent;
class QMouseEvent;
class QLabel;
class QTimer;
class QPushButton;
class QVBoxLayout;
class QGridLayout;
class QPaintEvent;

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

class KeyJoyPanel : public rviz_common::Panel
{
public:
  explicit KeyJoyPanel(QWidget * parent = nullptr);

  void onInitialize() override;

protected:
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;

private:
  void updatePublisher();
  void publishCurrent();
  void updatePublishRate();
  void updateStatusLabel();
  void refreshCommandFromInput();
  void setCommand(double axis_linear, double axis_angular);

  QVBoxLayout * main_layout_{};
  QLineEdit * topic_edit_{};
  QDoubleSpinBox * publish_rate_spin_{};
  QLabel * status_label_{};
  QTimer * publish_timer_{};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;

  std::string topic_ = "/joy";
  double publish_rate_hz_ = 10.0;

  sensor_msgs::msg::Joy last_msg_{};

  bool up_pressed_ = false;
  bool down_pressed_ = false;
  bool left_pressed_ = false;
  bool right_pressed_ = false;
};

class JoystickWidget : public QWidget
{
  Q_OBJECT

public:
  explicit JoystickWidget(QWidget * parent = nullptr);

  QSize minimumSizeHint() const override;

signals:
  void axisChanged(float axis_x, float axis_y, bool dragging);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;

private:
  void updateKnobPosition(const QPointF & position, bool dragging);
  QRectF drawRect() const;

  QPointF knob_pos_;
  bool dragging_ = false;
  float axis_x_ = 0.0f;
  float axis_y_ = 0.0f;
};

class ScreenJoyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ScreenJoyPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private slots:
  void onAxisChanged(float axis_x, float axis_y, bool dragging);

private:
  void updatePublisher();
  void publishCurrent();
  void updatePublishRate();
  void updateStatusLabel();
  void setCommand(float axis_x, float axis_y, bool dragging);

  QVBoxLayout * main_layout_{};
  QLineEdit * topic_edit_{};
  QDoubleSpinBox * publish_rate_spin_{};
  QLabel * status_label_{};
  QTimer * publish_timer_{};
  JoystickWidget * joystick_{};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;

  std::string topic_ = "/joy";
  double publish_rate_hz_ = 10.0;
  bool dragging_ = false;

  sensor_msgs::msg::Joy last_msg_{};
};

}  // namespace rviz2_teleop_plugin

#endif  // RVIZ2_TELEOP_PLUGIN__TELEOP_PANEL_HPP_
