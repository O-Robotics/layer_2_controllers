#ifndef AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "amr_sweeper_mission_executor/srv/end_mission.hpp"
#include "amr_sweeper_safety_msgs/msg/safety_stop.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace amr_sweeper_safety_controller
{

class SafetyControllerNode : public rclcpp::Node
{
public:
  explicit SafetyControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  void loadParameters();
  void onStopMessage(const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr msg);
  void onPublishTimer();

  void latchStop(const amr_sweeper_safety_msgs::msg::SafetyStop & stop_event);
  void clearLatchedStop();
  void publishZeroCommands();
  void publishDirectHardwareStopCommands();
  void publishStatus();
  void publishWebStatus();
  void requestMissionStop();
  void requestMotorStopPlaceholder();
  bool clearHardwareSafetyStops(std::string & failure_message);
  std::string buildWebStatusJson() const;

  void resetLatchedStopService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void enableControllerService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  bool enabled_{true};
  bool latched_stop_active_{false};
  bool mission_stop_requested_{false};
  bool mission_stop_placeholder_logged_{false};
  bool motor_stop_placeholder_logged_{false};

  double publish_rate_hz_{20.0};
  std::string stop_topic_name_{"safety_msgs/stop"};
  std::string wheel_stop_topic_{"cmd_vel_safety_stop"};
  std::string tool_stop_topic_{"cmd_vel_joy_tools"};
  bool publish_zero_tool_command_{true};
  bool publish_direct_hardware_stop_{true};
  std::string wheel_hardware_stop_topic_{"diff_cont/cmd_vel"};
  std::string tool_hardware_stop_topic_{"controller_steadydrive/commands"};
  bool mission_stop_enabled_{true};
  bool motor_stop_placeholder_enabled_{true};
  std::vector<std::string> future_motor_stop_interfaces_;
  std::string end_mission_service_name_{"end_mission"};
  std::vector<std::string> clear_safety_stop_service_names_{
    "/odrive_ros2_control/clear_safety_stop",
    "/steadydrive_ros2_control/clear_safety_stop"};
  std::string web_status_topic_{"safety_controller/web_status"};

  amr_sweeper_safety_msgs::msg::SafetyStop active_stop_event_;
  std::vector<amr_sweeper_safety_msgs::msg::SafetyStop> latched_stop_events_;

  rclcpp::Client<amr_sweeper_mission_executor::srv::EndMission>::SharedPtr end_mission_client_;
  std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clear_safety_stop_clients_;
  rclcpp::Subscription<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr stop_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_stop_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr tool_stop_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_hardware_stop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tool_hardware_stop_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_status_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_latched_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_safety_stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_controller_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace amr_sweeper_safety_controller

#endif  // AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_
