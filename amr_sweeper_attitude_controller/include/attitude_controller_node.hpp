#ifndef AMR_SWEEPER_ATTITUDE_CONTROLLER__AMR_SWEEPER_ATTITUDE_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_ATTITUDE_CONTROLLER__AMR_SWEEPER_ATTITUDE_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace amr_sweeper_attitude_controller
{

struct ImuMeasurement
{
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double weight{1.0};
};

struct AttitudeEstimate
{
  double roll_rad{0.0};
  double pitch_rad{0.0};
  bool healthy{false};
};

struct ImuInput
{
  std::string topic;
  double weight{1.0};
  sensor_msgs::msg::Imu last_msg;
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_received_time{0, 0, RCL_ROS_TIME};
  bool has_message{false};
  bool healthy{false};
  bool timeout_warning{false};
  bool timeout_error{false};
  std::string health_reason{"no messages received"};
};

struct ImuHealthConfig
{
  double timeout_warning_sec{0.3};
  double timeout_error_sec{1.0};
  double startup_grace_sec{0.0};
};

struct ImuHealth
{
  bool healthy{false};
  bool timeout_warning{false};
  bool timeout_error{false};
  std::string reason;
};

struct StopSupervisorOptions
{
  double roll_warning_deg{15.0};
  double pitch_warning_deg{15.0};
  double roll_stop_deg{30.0};
  double pitch_stop_deg{30.0};
  double nominal_roll_deg{0.0};
  double nominal_pitch_deg{5.0};
  bool require_manual_reset{true};
};

struct StopSupervisorState
{
  bool warning{false};
  bool stopped{false};
  bool latched{false};
  bool roll_warning{false};
  bool pitch_warning{false};
  bool roll_stop{false};
  bool pitch_stop{false};
  std::string reason{"ok"};
};

class StopSupervisor
{
public:
  explicit StopSupervisor(const StopSupervisorOptions & options = StopSupervisorOptions{});

  void setOptions(const StopSupervisorOptions & options);
  StopSupervisorState update(const AttitudeEstimate & attitude);
  bool resetFault();
  StopSupervisorState state() const;

private:
  StopSupervisorOptions options_;
  StopSupervisorState state_;
};

class AttitudeControllerNode : public rclcpp::Node
{
public:
  explicit AttitudeControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void configureImuInputs();
  void onImuMessage(sensor_msgs::msg::Imu::SharedPtr msg, std::size_t index);
  void onTimer();
  void publishLatestAttitudeDiagnostics();

  bool transformMeasurementToBaseLink(
    const sensor_msgs::msg::Imu & msg,
    ImuMeasurement * measurement,
    std::string * error_message);

  void publishAttitude(const rclcpp::Time & stamp, const AttitudeEstimate & estimate);
  void publishAttitudeDiagnostics(
    const rclcpp::Time & stamp,
    const AttitudeEstimate & estimate,
    std::size_t healthy_imu_count);
  void publishBaseLinkTransform(const rclcpp::Time & stamp, const AttitudeEstimate & estimate);
  void publishSafety(const rclcpp::Time & stamp, const StopSupervisorState & state);

  void resetFaultService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void enableAttitudeEstimationService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void enableSafetyStopService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool attitude_estimation_enabled_{true};
  bool safety_stop_enabled_{true};
  bool publish_base_link_tf_{true};
  bool publish_tool_link_tf_{false};
  bool hold_last_transform_when_unhealthy_{true};
  bool tool_link_warning_logged_{false};

  std::string base_footprint_frame_{"base_footprint"};
  std::string base_link_frame_{"base_link"};
  std::string tool_link_frame_{"tool_link"};
  std::string stop_topic_name_{"safety_msgs/stop"};

  double imu_timeout_warning_sec_{0.3};
  double imu_timeout_error_sec_{1.0};
  double imu_startup_grace_sec_{3.0};
  bool imu_timeout_stop_enabled_{true};
  double publish_rate_hz_{50.0};
  double status_publish_rate_hz_{2.0};
  double initial_roll_deg_{0.0};
  double initial_pitch_deg_{4.5};
  double base_link_origin_z_m_{0.13};

  std::vector<std::string> imu_topics_;
  std::vector<double> imu_weights_;
  std::vector<ImuInput> imu_inputs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;

  StopSupervisorOptions stop_options_;
  StopSupervisor stop_supervisor_;
  AttitudeEstimate last_estimate_;
  AttitudeEstimate last_transform_estimate_;
  std::size_t last_healthy_imu_count_{0};
  bool last_stop_request_active_{false};
  std::string last_stop_reason_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    attitude_diagnostics_publisher_;
  rclcpp::Publisher<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr stop_request_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_attitude_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_safety_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Time startup_time_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

bool isFinite(double value);
bool isFiniteVector(const geometry_msgs::msg::Vector3 & vector);
bool isZeroTime(const rclcpp::Time & stamp);
ImuHealth checkImuHealth(
  const ImuInput & input,
  const rclcpp::Time & now,
  const ImuHealthConfig & config);
double degreesToRadians(double degrees);
double radiansToDegrees(double radians);

}  // namespace amr_sweeper_attitude_controller

#endif  // AMR_SWEEPER_ATTITUDE_CONTROLLER__AMR_SWEEPER_ATTITUDE_CONTROLLER_NODE_HPP_
