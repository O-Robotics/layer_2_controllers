#ifndef AMR_SWEEPER_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_
#define AMR_SWEEPER_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace amr_sweeper_drive_controller
{

class DriveController : public controller_interface::ControllerInterface
{
public:
  DriveController() = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct TwistCommand
  {
    geometry_msgs::msg::Twist message;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct StampedCommand
  {
    geometry_msgs::msg::TwistStamped message;
    rclcpp::Time received_at{0, 0, RCL_ROS_TIME};
    bool valid{false};
  };

  struct OdometrySnapshot
  {
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    double x{0.0};
    double y{0.0};
    double heading{0.0};
    double linear_velocity{0.0};
    double angular_velocity{0.0};
    bool valid{false};
  };

  void resetCommandState();
  void resetOdometryState();
  void onWheelCommand(const geometry_msgs::msg::Twist::SharedPtr message);
  void onDirectCommand(const geometry_msgs::msg::TwistStamped::SharedPtr message);
  bool isFresh(const rclcpp::Time & now, const rclcpp::Time & received_at, double timeout_sec) const;
  void publishLatestOdometry();
  void publishOdometry(const OdometrySnapshot & snapshot);
  void publishOdometryTransform(const OdometrySnapshot & snapshot);
  void integrateWheelPositions(
    double left_position_rad, double right_position_rad, double dt_seconds);
  void integrateWheelVelocities(
    double left_velocity_rad_s, double right_velocity_rad_s, double dt_seconds);

  std::vector<std::string> left_wheel_names_{"LeftWheel_joint"};
  std::vector<std::string> right_wheel_names_{"RightWheel_joint"};
  std::string input_topic_{"cmd_vel_drive"};
  std::string direct_command_topic_{"drive_controller/cmd_vel"};
  std::string odom_topic_{"drive_controller/odom"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_footprint"};
  double wheel_separation_{0.490};
  double wheel_radius_{0.13};
  double left_wheel_radius_multiplier_{1.0};
  double right_wheel_radius_multiplier_{1.0};
  double wheel_separation_multiplier_{1.15};
  double command_timeout_sec_{0.5};
  double direct_command_timeout_sec_{0.5};
  double publish_rate_{10.0};
  bool position_feedback_{false};
  bool enable_odom_tf_{false};

  mutable std::mutex command_mutex_;
  TwistCommand latest_twist_command_;
  StampedCommand latest_direct_command_;
  mutable std::mutex odometry_mutex_;
  OdometrySnapshot latest_odometry_snapshot_;

  bool odometry_initialized_{false};
  double x_{0.0};
  double y_{0.0};
  double heading_{0.0};
  double linear_velocity_{0.0};
  double angular_velocity_{0.0};
  double previous_left_position_rad_{0.0};
  double previous_right_position_rad_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr direct_command_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher_;
  rclcpp::TimerBase::SharedPtr odom_publish_timer_;
};

}  // namespace amr_sweeper_drive_controller

#endif  // AMR_SWEEPER_DRIVE_CONTROLLER__DRIVE_CONTROLLER_HPP_
