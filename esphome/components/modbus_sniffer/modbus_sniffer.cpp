#include "modbus_sniffer.h"
#include "esphome/core/log.h"

namespace esphome::modbus_sniffer {

static const char *const TAG = "modbus_sniffer";

void ModbusSniffer::setup() {
  if (this->items_.empty())
    return;  // lambdas only: nothing to subscribe for
  this->parent_->add_on_response_callback(
      [this](uint8_t address, std::span<const uint8_t> request_pdu, std::span<const uint8_t> response_pdu) {
        this->publish_matching_(address, request_pdu, response_pdu);
      });
}

void ModbusSniffer::publish_matching_(uint8_t address, std::span<const uint8_t> request_pdu,
                                      std::span<const uint8_t> response_pdu) {
  const uint8_t function_code = request_pdu[0];

  // Only register reads put a start address in the request and registers in the reply. Coils pack
  // bits, and a write reply echoes its own address, so neither is a register an item can index.
  if (function_code != static_cast<uint8_t>(modbus::FunctionCode::READ_HOLDING_REGISTERS) &&
      function_code != static_cast<uint8_t>(modbus::FunctionCode::READ_INPUT_REGISTERS) &&
      function_code != static_cast<uint8_t>(modbus::FunctionCode::READ_WRITE_MULTIPLE_REGISTERS))
    return;

  // An exception reply carries an error code where the registers would be.
  if (modbus::helpers::is_function_code_exception(response_pdu[0]))
    return;

  if (request_pdu.size() < 3)
    return;
  const uint16_t start_address = modbus::helpers::get_data<uint16_t>(request_pdu.data(), 1);
  const std::span<const uint8_t> payload = modbus::helpers::server_pdu_payload(response_pdu);

  for (SnifferItem *item : this->items_) {
    if (item->address != address || item->reg < start_address)
      continue;
    const size_t index = static_cast<size_t>(item->reg - start_address) * 2;
    if (index + 1 >= payload.size())
      continue;  // this exchange did not reach that register
    item->parse_and_publish(static_cast<uint16_t>((payload[index] << 8) | payload[index + 1]));
  }
}

void ModbusSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus Sniffer:");
  ESP_LOGCONFIG(TAG, "  Items: %zu", this->items_.size());
  for (const SnifferItem *item : this->items_) {
    ESP_LOGCONFIG(TAG, "    address 0x%02X register %u", item->address, item->reg);
  }
}

}  // namespace esphome::modbus_sniffer
