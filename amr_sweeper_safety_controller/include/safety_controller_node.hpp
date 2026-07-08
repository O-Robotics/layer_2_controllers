#ifndef AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_
#define AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_

#include <cstdint>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
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
    std::vector<uint32_t> accepted_can_ids;
    int socket_fd{-1};
  };

  struct OdriveFeedbackState
  {
    uint32_t node_id{0U};
    bool heartbeat_seen{false};
    bool encoder_seen{false};
    uint32_t axis_error{0U};
    uint8_t axis_state{0U};
    double velocity_rev_s{0.0};
    rclcpp::Time last_heartbeat_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_encoder_time{0, 0, RCL_ROS_TIME};
  };

  struct SteadydriveFeedbackState
  {
    uint32_t can_id{0U};
    bool state_2_seen{false};
    double velocity_deg_s{0.0};
    rclcpp::Time last_state_2_time{0, 0, RCL_ROS_TIME};
  };

  void loadParameters();
  void onStopMessage(const amr_sweeper_safety_msgs::msg::SafetyStop::SharedPtr msg);
  void onPublishTimer();
  void noteProgress();
  void watchdogThreadMain();

  void latchStop(const amr_sweeper_safety_msgs::msg::SafetyStop & stop_event);
  void publishInternalStopEvent(const amr_sweeper_safety_msgs::msg::SafetyStop & stop_event);
  void clearLatchedStop();
  void publishZeroCommands();
  void publishDirectMotorStopCommands();
  void pollButtonCanFrames();
  void pollOdriveCanFrames();
  void pollSteadydriveCanFrames();
  void checkButtonHeartbeatWatchdog();
  void checkDirectMotorStopFeedback();
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
  bool sendCanRemoteFrame(
    CanInterfaceState & can_interface,
    uint32_t can_id,
    uint8_t payload_length,
    const std::string & description);
  bool sendOdriveEstopCommands();
  bool requestOdriveEncoderEstimates();
  bool sendSteadydriveStopCommands();
  bool requestSteadydriveState2();
  bool sendDirectMotorStopCommands(std::string & failure_message);
  bool parseButtonCanFrame(uint32_t can_id, const std::vector<uint8_t> & payload);
  bool parseOdriveCanFrame(uint32_t can_id, const std::vector<uint8_t> & payload);
  bool parseSteadydriveCanFrame(uint32_t can_id, const std::vector<uint8_t> & payload);
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
  bool direct_motor_stop_confirmed_{false};
  bool direct_motor_stop_confirmation_timeout_reported_{false};

  double publish_rate_hz_{20.0};
  std::string stop_topic_name_{"safety_msgs/stop"};
  std::string wheel_stop_topic_{"safety_controller/cmd_vel_safety_stop"};
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
  int stop_feedback_timeout_ms_{1500};
  int stop_feedback_stale_ms_{500};
  double odrive_stop_velocity_threshold_rev_s_{0.05};
  double steadydrive_stop_velocity_threshold_deg_s_{5.0};
  bool internal_watchdog_enabled_{true};
  int internal_watchdog_timeout_ms_{1500};
  int internal_watchdog_check_period_ms_{200};
  std::vector<std::string> clear_safety_stop_service_names_{
    "/odrive_ros2_control/clear_safety_stop",
    "/steadydrive_ros2_control/clear_safety_stop"};
  std::string web_status_topic_{"safety_controller/web_status"};
  std::string direct_motor_stop_last_failure_message_;
  std::string direct_motor_stop_confirmation_status_{"idle"};
  rclcpp::Time button_last_status_frame_time_{0, 0, RCL_ROS_TIME};
  uint16_t button_last_heartbeat_counter_{0U};
  bool button_heartbeat_seen_{false};
  rclcpp::Time latched_stop_since_{0, 0, RCL_ROS_TIME};

  amr_sweeper_safety_msgs::msg::SafetyStop active_stop_event_;
  std::vector<amr_sweeper_safety_msgs::msg::SafetyStop> latched_stop_events_;
  CanInterfaceState odrive_can_state_;
  CanInterfaceState steadydrive_can_state_;
  CanInterfaceState button_can_state_;
  CanInterfaceState odrive_feedback_can_state_;
  CanInterfaceState steadydrive_feedback_can_state_;
  std::vector<OdriveFeedbackState> odrive_feedback_states_;
  std::vector<SteadydriveFeedbackState> steadydrive_feedback_states_;
  std::atomic<bool> watchdog_running_{false};
  std::atomic<bool> internal_watchdog_stop_published_{false};
  std::atomic<long long> last_progress_time_ns_{0LL};
  std::thread watchdog_thread_;
  std::mutex can_tx_mutex_;

  rclcpp::Client<amr_sweeper_fsm::srv::RequestState>::SharedPtr fsm_request_client_;
  rclcpp::Client<amr_sweeper_mission_executor::srv::EndMission>::SharedPtr end_mission_client_;
  std::vector<rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> clear_safety_stop_clients_;
  rclcpp::Subscription<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr stop_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_stop_publisher_;
  rclcpp::Publisher<amr_sweeper_safety_msgs::msg::SafetyStop>::SharedPtr stop_event_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr web_status_publisher_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_latched_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_safety_stop_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_controller_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace amr_sweeper_safety_controller

#endif  // AMR_SWEEPER_SAFETY_CONTROLLER__AMR_SWEEPER_SAFETY_CONTROLLER_NODE_HPP_
