#pragma once

#include "esphome/core/component.h"
#include "esphome/components/modbus/modbus.h"

#include <cstdint>
#include <span>
#include <vector>

namespace esphome::modbus_sniffer {

/** One register a platform wants from the bus.
 *
 * Kept abstract so this component carries no dependency on any platform: a sensor is one
 * implementation, and a text sensor or binary sensor would be others.
 */
class SnifferItem {
 public:
  SnifferItem(uint8_t address, uint16_t reg) : reg(reg), address(address) {}
  virtual ~SnifferItem() = default;

  /// The register's two bytes, big-endian, from a response that covered it.
  virtual void parse_and_publish(uint16_t raw) = 0;

  uint16_t reg;
  uint8_t address;
};

/** Attaches behaviour to a hub running `role: sniffer`.
 *
 * The hub pairs requests with replies and knows nothing about what they mean. This is where that
 * meaning lives: `on_request`/`on_response` lambdas, and items that name a register and get
 * published when an exchange covering it goes past.
 */
class ModbusSniffer : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(modbus::ModbusSnifferHub *parent) { this->parent_ = parent; }
  void add_item(SnifferItem *item) { this->items_.push_back(item); }

 protected:
  void publish_matching_(uint8_t address, std::span<const uint8_t> request_pdu, std::span<const uint8_t> response_pdu);

  std::vector<SnifferItem *> items_;
  modbus::ModbusSnifferHub *parent_{nullptr};
};

}  // namespace esphome::modbus_sniffer
