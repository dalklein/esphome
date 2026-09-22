#pragma once

#include "esphome/components/modbus_sniffer/modbus_sniffer.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome::modbus_sniffer {

class ModbusSnifferSensor final : public sensor::Sensor, public SnifferItem {
 public:
  ModbusSnifferSensor(uint8_t address, uint16_t reg, bool is_signed)
      : SnifferItem(address, reg), is_signed_(is_signed) {}

  /// S_WORD reinterprets the same 16 bits as int16_t: a discharging battery reports a negative
  /// power that would otherwise read as ~65000.
  void parse_and_publish(uint16_t raw) override {
    this->publish_state(this->is_signed_ ? static_cast<float>(static_cast<int16_t>(raw)) : static_cast<float>(raw));
  }

 protected:
  bool is_signed_;
};

}  // namespace esphome::modbus_sniffer
