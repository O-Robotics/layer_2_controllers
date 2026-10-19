#ifndef AMR_SWEEPER_SAFETY_CONTROLLER__BUTTON_CAN_PROTOCOL_HPP_
#define AMR_SWEEPER_SAFETY_CONTROLLER__BUTTON_CAN_PROTOCOL_HPP_

#include <chrono>
#include <cstdint>
#include <vector>

namespace amr_sweeper_safety_controller
{
namespace button_can
{

// Matches canonical Button firmware: module_can.c, heartbeat.c and Button.c.
constexpr uint8_t kPressedEvent = 0x01U;
constexpr uint8_t kRunningStatus = 0x02U;  // MODULE_STATUS_OK
constexpr uint16_t kStopFlags = (1U << 0U) | (1U << 4U) | (1U << 8U);

struct StatusFrame
{
  uint16_t firmware_version{0U};
  uint8_t module_status{0U};
  uint16_t flags{0U};
  uint8_t detail{0U};
  uint8_t rx_overflow{0U};
  uint8_t extended_rx_overflow{0U};

  bool stopAsserted() const {return (flags & kStopFlags) != 0U;}
  bool healthy() const {return module_status == kRunningStatus;}
};

inline bool isPressedEvent(
  const uint32_t can_id, const uint32_t base_id, const std::vector<uint8_t> & payload)
{
  return base_id <= 0x7FDU && can_id == (base_id + 2U) &&
         payload.size() >= 2U && payload[0] == kPressedEvent;
}

inline bool parseStatusFrame(
  const uint32_t can_id, const uint32_t base_id,
  const std::vector<uint8_t> & payload, StatusFrame & result)
{
  if (base_id > 0x7FDU || can_id != (base_id + 1U) || payload.size() != 8U ||
    payload[2] > 3U)
  {
    return false;
  }
  result.firmware_version = static_cast<uint16_t>(payload[0]) |
    (static_cast<uint16_t>(payload[1]) << 8U);
  result.module_status = payload[2];
  result.flags = static_cast<uint16_t>(payload[3]) |
    (static_cast<uint16_t>(payload[4]) << 8U);
  result.detail = payload[5];
  result.rx_overflow = payload[6];
  result.extended_rx_overflow = payload[7];
  return true;
}

inline bool isFresh(
  const bool heartbeat_seen, const std::chrono::steady_clock::duration elapsed,
  const std::chrono::milliseconds maximum_age)
{
  return heartbeat_seen && elapsed >= std::chrono::steady_clock::duration::zero() &&
         elapsed <= maximum_age;
}

}  // namespace button_can
}  // namespace amr_sweeper_safety_controller

#endif  // AMR_SWEEPER_SAFETY_CONTROLLER__BUTTON_CAN_PROTOCOL_HPP_
