#include "drivecontrol/can_interface.hpp"

#include <cmath>
#include <cstring>

namespace dc {

uint8_t crc8_j1850(const CanFrame& f) {
  uint8_t crc = 0xFF;
  auto feed = [&crc](uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; ++i) crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x1D) : static_cast<uint8_t>(crc << 1);
  };
  feed(static_cast<uint8_t>(f.id & 0xFF));
  feed(static_cast<uint8_t>((f.id >> 8) & 0xFF));
  for (int i = 0; i < 7; ++i) feed(f.data[static_cast<size_t>(i)]);
  return static_cast<uint8_t>(crc ^ 0xFF);
}

void seal(CanFrame& f, uint8_t counter) {
  f.data[6] = static_cast<uint8_t>((f.data[6] & 0xF0) | (counter & 0x0F));
  f.data[7] = crc8_j1850(f);
}

bool check(const CanFrame& f) { return f.len == 8 && f.data[7] == crc8_j1850(f); }

namespace {
void put_u16(CanFrame& f, int at, uint16_t v) {
  f.data[static_cast<size_t>(at)] = static_cast<uint8_t>(v & 0xFF);
  f.data[static_cast<size_t>(at + 1)] = static_cast<uint8_t>(v >> 8);
}
uint16_t get_u16(const CanFrame& f, int at) {
  return static_cast<uint16_t>(f.data[static_cast<size_t>(at)] | (f.data[static_cast<size_t>(at + 1)] << 8));
}
}  // namespace

CanFrame encode(const CommandMsg& m, uint8_t counter) {
  CanFrame f;
  f.id = canid::kCommand;
  put_u16(f, 0, static_cast<uint16_t>(clamp(std::lround(m.target_speed * 100.0), 0L, 65535L)));
  f.data[2] = static_cast<uint8_t>((m.enable ? 1 : 0) | (m.reset_request ? 2 : 0));
  seal(f, counter);
  return f;
}

CanFrame encode(const EStopMsg& m, uint8_t counter) {
  CanFrame f;
  f.id = canid::kEStop;
  f.data[0] = m.estop ? 1 : 0;
  seal(f, counter);
  return f;
}

CanFrame encode(const StatusMsg& m, uint8_t counter) {
  CanFrame f;
  f.id = canid::kStatus;
  f.data[0] = m.state;
  put_u16(f, 1, static_cast<uint16_t>(clamp(std::lround(m.speed * 100.0), 0L, 65535L)));
  put_u16(f, 3, static_cast<uint16_t>(static_cast<int16_t>(std::lround(m.steer * 10000.0))));
  f.data[5] = m.faults;
  seal(f, counter);
  return f;
}

CanFrame encode(const ActuatorCommand& m, uint8_t counter) {
  CanFrame f;
  f.id = canid::kActuatorCmd;
  put_u16(f, 0, static_cast<uint16_t>(static_cast<int16_t>(std::lround(m.steer * 10000.0))));
  f.data[2] = static_cast<uint8_t>(std::lround(clamp(m.throttle, 0, 1) * 200.0));
  f.data[3] = static_cast<uint8_t>(std::lround(clamp(m.brake, 0, 1) * 200.0));
  seal(f, counter);
  return f;
}

bool decode(const CanFrame& f, CommandMsg& m) {
  if (f.id != canid::kCommand || !check(f)) return false;
  m.target_speed = get_u16(f, 0) / 100.0;
  m.enable = f.data[2] & 1;
  m.reset_request = f.data[2] & 2;
  return true;
}

bool decode(const CanFrame& f, EStopMsg& m) {
  if (f.id != canid::kEStop || !check(f)) return false;
  m.estop = f.data[0] & 1;
  return true;
}

bool decode(const CanFrame& f, StatusMsg& m) {
  if (f.id != canid::kStatus || !check(f)) return false;
  m.state = f.data[0];
  m.speed = get_u16(f, 1) / 100.0;
  m.steer = static_cast<int16_t>(get_u16(f, 3)) / 10000.0;
  m.faults = f.data[5];
  return true;
}

bool decode(const CanFrame& f, ActuatorCommand& m) {
  if (f.id != canid::kActuatorCmd || !check(f)) return false;
  m.steer = static_cast<int16_t>(get_u16(f, 0)) / 10000.0;
  m.throttle = f.data[2] / 200.0;
  m.brake = f.data[3] / 200.0;
  return true;
}

bool RxSupervisor::accept(const CanFrame& f, double t) {
  if (!check(f)) {
    ++rejected_;
    return false;
  }
  const uint8_t c = f.data[6] & 0x0F;
  if (have_ctr_) {
    const uint8_t d = static_cast<uint8_t>((c - ctr_) & 0x0F);
    ctr_ = c;
    if (d == 0 || d > 2) {
      ++rejected_;
      return false;
    }
  }
  ctr_ = c;
  have_ctr_ = true;
  last_ok_ = t;
  ever_ = true;
  return true;
}

}  // namespace dc
