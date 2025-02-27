#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <vector>

namespace esphome {
namespace modbus {

static const char *const TAG = "modbus";  

enum ModbusRole {
  CLIENT,
  SERVER,
  MUTE_CLIENT,
};   
// MUTE_CLIENT is a passive listener, 
//  reading data returned by another server in response to the bus CLIENT (master),
//  after observing a CLIENT request matching addresses configured for the MUTE_CLIENT sensors.
//   MUTE_CLIENT is not a true Modbus device, and does not respond to Modbus commands.

enum frame_type_enum { error_80, response_custom, response_0304, command_0304, response_05060F10, command_05060F10,no_frame};

class ModbusDevice;


class Modbus : public uart::UARTDevice, public Component {
 public:
  Modbus() = default;

  void setup() override;

  void loop() override;

  void dump_config() override;

  void register_device(ModbusDevice *device) { this->devices_.push_back(device); }

  float get_setup_priority() const override;

  void send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
            uint8_t payload_len = 0, const uint8_t *payload = nullptr,bool disable_send=false);
  void send_raw(const std::vector<uint8_t> &payload,bool disable_send=false);
  void set_role(ModbusRole role) { this->role = role; }
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  uint8_t waiting_for_response{0};
  void set_send_wait_time(uint16_t time_in_ms) { send_wait_time_ = time_in_ms; }
  void set_disable_crc(bool disable_crc) { disable_crc_ = disable_crc; }

  ModbusRole role;

 protected:
  GPIOPin *flow_control_pin_{nullptr};

  bool parse_modbus_byte_(uint8_t byte);
  uint16_t send_wait_time_{250};
  bool disable_crc_;
  std::vector<uint8_t> rx_buffer_;
  uint32_t last_modbus_byte_{0};
  uint32_t last_send_{0};
  std::vector<ModbusDevice *> devices_;
};

class ModbusDevice {
 public:
  void set_parent(Modbus *parent) { parent_ = parent; }
  void set_address(uint8_t address) { address_ = address; }
//  void set_disable_send(bool disable_send) { disable_send_ = disable_send; }
//  bool get_disable_send() const { return disable_send_; } 
  ModbusDevice() : disable_send_(false) {
    ESP_LOGW(TAG, "ModbusDevice constructor, disable_send_ initialized to %d", disable_send_);
  }
  void set_disable_send(bool disable_send) { 
    ESP_LOGW(TAG, "ModbusDevice::set_disable_send to %d for device 0x%02X", disable_send, address_); 
    disable_send_ = disable_send; 
  }
  bool get_disable_send() const { 
//    ESP_LOGW(TAG, "ModbusDevice::get_disable_send returns %d for device 0x%02X", disable_send_, address_);
    return disable_send_; 
  }

  uint8_t get_address() const { return address_; }
//  ModbusRole role_{ModbusRole::CLIENT}; 
//  virtual void on_modbus_data(const std::vector<uint8_t> &data) = 0;
  virtual void on_modbus_data(bool is_response,uint8_t address,uint8_t function_code, uint16_t start_address,uint16_t number_of_registers,uint16_t crc,const std::vector<uint8_t> &data)= 0;
  virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}
  virtual void on_modbus_read_registers(uint8_t function_code, uint16_t start_address, uint16_t number_of_registers){ };
  virtual void on_modbus_read_registers_mute(uint8_t function_code, uint16_t start_address, uint16_t number_of_registers){ };
  void send(uint8_t function, uint16_t start_address, uint16_t number_of_entities, uint8_t payload_len = 0,
            const uint8_t *payload = nullptr,bool disable_send=false) {
    this->parent_->send(this->address_, function, start_address, number_of_entities, payload_len, payload,disable_send);
  }
  void send_raw(const std::vector<uint8_t> &payload,bool disable_send=false) { this->parent_->send_raw(payload,disable_send); }
  // If more than one device is connected block sending a new command before a response is received
  bool waiting_for_response() { return parent_->waiting_for_response != 0; }

 protected:
  friend Modbus;

  Modbus *parent_;
  uint8_t address_;
  bool disable_send_;  

};

}  // namespace modbus
}  // namespace esphome
