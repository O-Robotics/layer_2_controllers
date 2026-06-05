#ifndef AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "amr_sweeper_mission_executor/srv/end_mission.hpp"
#include "amr_sweeper_fsm/srv/request_state.hpp"
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
  ~SafetyControllerNode() override;

private:
  struct CanInterfaceState
  {
    std::string interface_name;
    int socket_fd{-1};
  };

  void loadParameters();
  void onStopMessage(const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr msg);
  void onPublishTimer();

  void latchStop(const amr_sweeper_safety_msgs::msg::SafetyStop & stop_event);
  void clearLatchedStop();
  void publishZeroCommands();
  void publishDirectHardwareStopCommands();
  void publishDirectMotorStopCommands();
  void pollButtonCanFrames();
  void checkButtonHeartbeatWatchdog();
  void publishStatus();
  void publishWebStatus();
  void requestMissionStop();
  void requestFsmFaultState();
  bool ensureCanInterface(CanInterfaceState & can_interface, const std::string & description);
  void closeCanInterface(CanInterfaceState & can_interface);
  bool sendCanFrame(
    CanInterfaceState & can_interface,
    uint32_t can_id,
    const std::vector<uint8_t> & payload,
    const std::string & description);
  bool sendOdriveEstopCommands();
  bool sendSteadydriveStopCommands();
  bool sendDirectMotorStopCommands(std::string & failure_message);
  bool parseButtonCanFrame(uint32_t can_id, const std::vector<uint8_t> & payload);
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
  bool fsm_fault_requested_{false};
  bool mission_stop_placeholder_logged_{false};
  bool direct_motor_stop_failure_logged_{false};
  bool direct_motor_stop_healthy_{true};

  double publish_rate_hz_{20.0};
  std::string stop_topic_name_{"safety_msgs/stop"};
  std::string wheel_stop_topic_{"cmd_vel_safety_stop"};
  std::string tool_stop_topic_{"cmd_vel_joy_tools"};
  bool publish_zero_tool_command_{true};
  bool publish_direct_hardware_stop_{true};
  std::string wheel_hardware_stop_topic_{"drive_controller/cmd_vel"};
  std::string tool_hardware_stop_topic_{"tool_controller/commands"};
  bool mission_stop_enabled_{true};
  bool direct_can_motor_stop_enabled_{true};
  bool odrive_direct_can_stop_enabled_{true};
  std::string odrive_can_interface_{"can0"};
  std::vector<uint32_t> odrive_node_ids_{0U, 2U};
  bool steadydrive_direct_can_stop_enabled_{true};
  std::string steadydrive_can_interface_{"can0"};
  std::vector<uint32_t> steadydrive_motor_can_ids_{0x141U, 0x142U};
  bool button_can_monitor_enabled_{true};
  std::string button_can_interface_{"can0"};
  uint32_t button_can_base_id_{0x200U};
  int button_status_period_ms_{5000};
  int button_heartbeat_timeout_ms_{12000};
  std::string fsm_request_service_name_{"request_state"};
  uint16_t fsm_fault_profile_id_{400U};
  uint8_t fsm_fault_request_priority_{255U};
  std::string end_mission_service_name_{"end_mission"};
  std::vector<std::string> clear_safety_stop_service_names_{
    "/odrive_ros2_control/clear_safety_stop",
    "/steadydrive_ros2_control/clear_safety_stop"};
  std::string web_status_topic_{"safety_controller/web_status"};
  std::string direct_motor_stop_last_failure_message_;
  rclcpp::Time button_last_status_frame_time_{0, 0, RCL_ROS_TIME};
  uint16_t button_last_heartbeat_counter_{0U};
  bool button_heartbeat_seen_{false};

  amr_sweeper_safety_msgs::msg::SafetyStop active_stop_event_;
  std::vector<amr_sweeper_safety_msgs::msg::SafetyStop> latched_stop_events_;
  CanInterfaceState odrive_can_state_;
  CanInterfaceState steadydrive_can_state_;
  CanInterfaceState button_can_state_;

  rclcpp::Client<amr_sweeper_fsm::srv::RequestState>::SharedPtr fsm_request_client_;
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
