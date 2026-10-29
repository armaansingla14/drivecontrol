#pragma once
#include <array>
#include <cstdint>

#include "drivecontrol/types.hpp"

namespace dc {

struct CanFrame {
  uint32_t id = 0;
  uint8_t len = 8;
  std::array<uint8_t, 8> data{};
};

/// CAN message set of the drive controller (classic CAN, 500 kbit/s).
/// Every frame carries a 4-bit alive counter (byte 6, low nibble) and a
/// CRC-8 SAE J1850 over the ID and bytes 0..6 (byte 7).
namespace canid {
constexpr uint32_t kEStop = 0x080;        ///< safety controller -> DC, 20 ms heartbeat
constexpr uint32_t kCommand = 0x100;      ///< planner -> DC, 50 ms
constexpr uint32_t kStatus = 0x300;       ///< DC -> planner, 20 ms
constexpr uint32_t kActuatorCmd = 0x310;  ///< DC -> actuator ECUs, 10 ms
}  // namespace canid

uint8_t crc8_j1850(const CanFrame& f);
void seal(CanFrame& f, uint8_t counter);
bool check(const CanFrame& f);

struct CommandMsg {
  double target_speed = 0;  ///< m/s, 0.01 resolution
  bool enable = false;
  bool reset_request = false;
};
struct EStopMsg {
  bool estop = false;
};
struct StatusMsg {
  uint8_t state = 0;
  double speed = 0;
  double steer = 0;
  uint8_t faults = 0;
};

CanFrame encode(const CommandMsg& m, uint8_t counter);
CanFrame encode(const EStopMsg& m, uint8_t counter);
CanFrame encode(const StatusMsg& m, uint8_t counter);
CanFrame encode(const ActuatorCommand& m, uint8_t counter);
bool decode(const CanFrame& f, CommandMsg& m);
bool decode(const CanFrame& f, EStopMsg& m);
bool decode(const CanFrame& f, StatusMsg& m);
bool decode(const CanFrame& f, ActuatorCommand& m);

/// Receive supervision for one periodic message: CRC, alive counter
/// (repeats and jumps > 2 rejected) and timeout.
class RxSupervisor {
 public:
  explicit RxSupervisor(double timeout_s) : timeout_(timeout_s) {}
  bool accept(const CanFrame& f, double t);  ///< true if the payload may be used
  bool timed_out(double t) const { return !ever_ || t - last_ok_ > timeout_; }
  bool ever_received() const { return ever_; }
  uint32_t rejected() const { return rejected_; }

 private:
  double timeout_;
  double last_ok_ = 0;
  bool ever_ = false;
  bool have_ctr_ = false;
  uint8_t ctr_ = 0;
  uint32_t rejected_ = 0;
};

}  // namespace dc
