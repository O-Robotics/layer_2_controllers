#ifndef AMR_SWEEPER_ATTITUDE_CONTROLLER__ATTITUDE_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_ATTITUDE_CONTROLLER__ATTITUDE_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "attitude_estimator.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "imu_input.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "stop_supervisor.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_sweeper_attitude_controller
{

class AttitudeControllerNode : public rclcpp::Node
{
public:
  explicit AttitudeControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void configureImuInputs();
  void onImuMessage(sensor_msgs::msg::Imu::SharedPtr msg, std::size_t index);
  void onTimer();

  bool transformMeasurementToBaseLink(
    const sensor_msgs::msg::Imu & msg,
    ImuMeasurement * measurement,
    std::string * error_message);

  void publishAttitude(const rclcpp::Time & stamp, const AttitudeEstimate & estimate);
  void publishAttitudeDiagnostics(
    const rclcpp::Time & stamp,
    const AttitudeEstimate & estimate,
    std::size_t healthy_imu_count);
  void publishBaseLinkJointStates(const rclcpp::Time & stamp, const AttitudeEstimate & estimate);
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
  bool safety_stop_enabled_{false};
  bool publish_base_link_joint_states_{true};
  bool publish_tool_link_tf_{false};
  bool tool_link_warning_logged_{false};

  std::string base_footprint_frame_{"base_footprint"};
  std::string base_link_frame_{"base_link"};
  std::string tool_link_frame_{"tool_link"};
  std::string base_roll_joint_name_{"base_roll_joint"};
  std::string base_pitch_joint_name_{"base_pitch_joint"};
  std::string stop_topic_name_{"safety_msgs/stop"};

  double imu_timeout_sec_{0.25};
  double publish_rate_hz_{50.0};

  std::vector<std::string> imu_topics_;
  std::vector<double> imu_weights_;
  std::vector<ImuInput> imu_inputs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> imu_subscriptions_;

  AttitudeEstimatorOptions estimator_options_;
  StopSupervisorOptions stop_options_;
  AttitudeEstimator estimator_;
  StopSupervisor stop_supervisor_;
  AttitudeEstimate last_estimate_;
  bool last_stop_request_active_{false};
  std::string last_stop_reason_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    attitude_diagnostics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr base_joint_state_publisher_;
  rclcpp::Publisher<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr stop_request_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_attitude_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_safety_service_;

  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace amr_sweeper_attitude_controller

#endif  // AMR_SWEEPER_ATTITUDE_CONTROLLER__ATTITUDE_CONTROLLER_NODE_HPP_
