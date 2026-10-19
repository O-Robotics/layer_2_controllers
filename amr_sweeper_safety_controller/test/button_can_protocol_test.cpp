#include <cassert>
#include <chrono>
#include <cstdint>
#include <vector>

#include "button_can_protocol.hpp"

using amr_sweeper_safety_controller::button_can::StatusFrame;
using amr_sweeper_safety_controller::button_can::isFresh;
using amr_sweeper_safety_controller::button_can::isPressedEvent;
using amr_sweeper_safety_controller::button_can::parseStatusFrame;

int main()
{
  constexpr uint32_t base = 0x200U;
  assert(isPressedEvent(base + 2U, base, {0x01, 0x00}));
  assert(!isPressedEvent(base, base, {0x01, 0x00}));
  assert(!isPressedEvent(base + 2U, base, {0x02, 0x00}));
  assert(!isPressedEvent(base + 2U, base, {0x01}));

  StatusFrame status;
  const std::vector<uint8_t> healthy{0x15, 0x42, 0x02, 0, 0, 0, 0xFF, 0x81};
  assert(parseStatusFrame(base + 1U, base, healthy, status));
  assert(status.healthy());
  assert(!status.stopAsserted());
  assert(status.rx_overflow == 0xFF && status.extended_rx_overflow == 0x81);
  assert(status.firmware_version == 0x4215);

  auto pressed = healthy;
  pressed[3] = 0x01;
  assert(parseStatusFrame(base + 1U, base, pressed, status) && status.stopAsserted());
  pressed[3] = 0x10;
  assert(parseStatusFrame(base + 1U, base, pressed, status) && status.stopAsserted());
  pressed[3] = 0x00;
  pressed[4] = 0x01;
  assert(parseStatusFrame(base + 1U, base, pressed, status) && status.stopAsserted());

  pressed[4] = 0;
  pressed[2] = 0;
  assert(parseStatusFrame(base + 1U, base, pressed, status) && !status.healthy());
  pressed[2] = 3;
  assert(parseStatusFrame(base + 1U, base, pressed, status) && !status.healthy());
  pressed[2] = 4;
  assert(!parseStatusFrame(base + 1U, base, pressed, status));
  assert(!parseStatusFrame(base, base, healthy, status));
  assert(!parseStatusFrame(base + 1U, base, {1, 2, 3}, status));

  using namespace std::chrono;
  assert(!isFresh(false, milliseconds(0), milliseconds(12000)));
  assert(isFresh(true, milliseconds(12000), milliseconds(12000)));
  assert(!isFresh(true, milliseconds(12001), milliseconds(12000)));
  assert(!isFresh(true, milliseconds(-1), milliseconds(12000)));
  return 0;
}
