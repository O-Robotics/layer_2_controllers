#ifndef AMR_SWEEPER_COLLISION_DETECTOR__COLLISION_DETECTOR_NODE_HPP_
#define AMR_SWEEPER_COLLISION_DETECTOR__COLLISION_DETECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_sweeper_collision_detector
{

struct ImuInput
{
  std::string name;
  std::string topic;
  double acceleration_threshold_mps2{14.0};
  sensor_msgs::msg::Imu last_msg;
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_received_time{0, 0, RCL_ROS_TIME};
  bool has_message{false};
};

struct MotorForceInput
{
  std::string name;
  std::string topic;
  std::string joint_name;
  double effort_threshold{5.0};
  double last_effort{0.0};
  rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_received_time{0, 0, RCL_ROS_TIME};
  bool has_message{false};
};

struct SourceHealth
{
  bool healthy{false};
  bool warning{false};
  bool error{false};
  std::string reason{"not configured"};
};

class CollisionDetectorNode : public rclcpp::Node
{
public:
  explicit CollisionDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void configureImuInputs();
  void configureMotorForceInputs();

  void onImuMessage(sensor_msgs::msg::Imu::SharedPtr msg, std::size_t index);
  void onMotorForceMessage(sensor_msgs::msg::JointState::SharedPtr msg, std::size_t index);
  void onTimer();
  void publishLatestDiagnostics();

  void publishDiagnostics(
    const rclcpp::Time & stamp,
    const std::vector<SourceHealth> & imu_health,
    const std::vector<SourceHealth> & motor_health,
    std::size_t healthy_imu_count,
    bool impact_detected,
    const std::string & impact_reason);
  void publishImpactState(bool impact_detected, const std::string & impact_reason);
  void resetImpactLatchService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void enableDetectorService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool enabled_{true};
  bool latch_collision_{true};
  bool publish_stop_request_{true};
  double publish_rate_hz_{20.0};
  double status_publish_rate_hz_{2.0};
  double imu_timeout_warning_sec_{0.3};
  double imu_timeout_error_sec_{1.0};
  double motor_timeout_warning_sec_{0.5};
  double motor_timeout_error_sec_{1.5};

  std::string impact_state_topic_{"collision_detector/impact_detected"};
  std::string stop_topic_name_{"safety_msgs/stop"};
  std::string stop_sender_name_{"collision_detector"};

  std::vector<std::string> configured_imu_names_;
  std::vector<std::string> configured_motor_force_names_;

  std::vector<ImuInput> imu_inputs_;
  std::vector<MotorForceInput> motor_force_inputs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> motor_force_subscriptions_;

  bool impact_latched_{false};
  bool last_reported_impact_state_{false};
  std::string latched_impact_reason_{"no impact detected"};

  std::vector<SourceHealth> last_imu_health_;
  std::vector<SourceHealth> last_motor_health_;
  std::size_t last_healthy_imu_count_{0};
  bool last_reported_impact_{false};
  std::string last_reported_reason_{"no impact detected"};

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr impact_state_publisher_;
  rclcpp::Publisher<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr stop_request_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_impact_latch_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_detector_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

SourceHealth evaluateImuHealth(
  const ImuInput & input,
  const rclcpp::Time & now,
  double timeout_warning_sec,
  double timeout_error_sec);
SourceHealth evaluateMotorForceHealth(
  const MotorForceInput & input,
  const rclcpp::Time & now,
  double timeout_warning_sec,
  double timeout_error_sec);
bool isFiniteVector3(const geometry_msgs::msg::Vector3 & vector);
double magnitude(const geometry_msgs::msg::Vector3 & vector);
bool isZeroTime(const rclcpp::Time & stamp);

}  // namespace amr_sweeper_collision_detector

#endif  // AMR_SWEEPER_COLLISION_DETECTOR__COLLISION_DETECTOR_NODE_HPP_
