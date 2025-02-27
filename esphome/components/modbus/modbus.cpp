#include "modbus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace modbus {

//static const char *const TAG = "modbus";

void Modbus::setup() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
  }
}
void Modbus::loop() {
  const uint32_t now = millis();

  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    if (this->parse_modbus_byte_(byte)) {
      this->last_modbus_byte_ = now;
    } else {
      size_t at = this->rx_buffer_.size();
      if (at > 0) {
        ESP_LOGV(TAG, "Clearing buffer of %d bytes - parse failed", at);
        this->rx_buffer_.clear();
      }
    }
  }

  if (now - this->last_modbus_byte_ > 50) {
    size_t at = this->rx_buffer_.size();
    if (at > 0) {
      ESP_LOGV(TAG, "Clearing buffer of %d bytes - timeout", at);
      this->rx_buffer_.clear();
    }

    // stop blocking new send commands after sent_wait_time_ ms after response received
    if (now - this->last_send_ > send_wait_time_) {
      if (waiting_for_response > 0) {
        ESP_LOGV(TAG, "Stop waiting for response from %d", waiting_for_response);
      }
      waiting_for_response = 0;
    }
  }
}

bool Modbus::parse_modbus_byte_(uint8_t byte) {
  static const size_t MAX_MESSAGE_SIZE = 512;
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  const uint8_t *raw = &this->rx_buffer_[0];
  ESP_LOGVV(TAG, "Modbus received Byte  %d (0X%x)", byte, byte);
  // Byte 0: modbus address (match all)
  if (at == 0)
    return true;
  uint8_t address = raw[0];
  uint8_t function_code = raw[1];
  // Byte 2: Size (with modbus rtu function code 4/3)
  // See also https://en.wikipedia.org/wiki/Modbus
  if (at <= 2)   //was == 2
    return true;

  // Modbus frame types, parsing will determine and set frame_type
  //   and data_len, data_offset, is_response are looked up by frame_type
  frame_type_enum frame_type=no_frame;
  unsigned int data_len[6];
  unsigned int data_offset[6];
  bool is_response[6];
  data_len[response_custom]=at - 2;
  data_offset[response_custom]=1;
  is_response[response_custom]=true;
  data_len[error_80]=1;
  data_offset[error_80]=2;
  is_response[error_80]=true;
  data_len[command_0304]=4;
  data_offset[command_0304]=2;
  // passed as flag to on_modbus_data, to disable_send.  
  is_response[command_0304]=false;  // if the frame is command, then false. don't don't reply, ie: do reply 
  data_len[response_05060F10]=4;
  data_offset[response_05060F10]=2;
  is_response[response_05060F10]=true;
  data_len[response_0304]=raw[2];
  data_offset[response_0304]=3;
  is_response[response_0304]=true; // if frame id'd as response, then true, do disable_send, and parse the rec'd data.
  data_len[command_05060F10]=raw[6];
  data_offset[command_05060F10]=7;
  is_response[command_05060F10]=false;

  // Per https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf Ch 5 User-Defined function codes
  if (((function_code >= 65) && (function_code <= 72)) || ((function_code >= 100) && (function_code <= 110))) {
    // Handle user-defined function, since we don't know how big this ought to be,
    // ideally we should delegate the entire length detection to whatever handler is
    // installed, but wait, there is the CRC, and if we get a hit there is a good
    // chance that this is a complete message ... admittedly there is a small chance is
    // isn't but that is quite small given the purpose of the CRC in the first place

    // Fewer than 2 bytes can't calc CRC
    if (at < 2)
      return true;
    // more than 2 bytes, calc CRC. 
    frame_type = response_custom;    // note, for resp_cust, data_len is set to at-2
    uint16_t computed_crc = crc16(raw, data_offset[frame_type] + data_len[frame_type]);
    uint16_t remote_crc = uint16_t(raw[data_offset[frame_type] + data_len[frame_type]]) | (uint16_t(raw[data_offset[frame_type] + data_len[frame_type] + 1]) << 8);

    if (computed_crc != remote_crc)
      return true;

    ESP_LOGD(TAG, "Modbus user-defined function %02X found", function_code);

  //} else {
    // data starts at 2 and length is 4 for read registers commands
    //if ((this->role == ModbusRole::SERVER || this->role == ModbusRole::MUTE_CLIENT) && (function_code == 0x3 || function_code == 0x4)) {
    //  data_offset = 2;
    //  data_len = 4;
    //}
  }
  else if ((function_code == 0x3 || function_code == 0x4)) {
    //check for command
    if ((at > data_offset[command_0304]+data_len[command_0304])) { //read commands are 8 bytes      
      uint16_t computed_crc = crc16(raw, data_offset[command_0304] + data_len[command_0304]);
      uint16_t remote_crc = uint16_t(raw[data_offset[command_0304] + data_len[command_0304]]) | (uint16_t(raw[data_offset[command_0304] + data_len[command_0304] + 1]) << 8);
      if (computed_crc == remote_crc)  {
        frame_type = command_0304;
      }
    }
    //check for response
    if ((at > data_offset[response_0304]+data_len[response_0304]) && (frame_type==no_frame)) {
      uint16_t computed_crc = crc16(raw, data_offset[response_0304] + data_len[response_0304]);
      uint16_t remote_crc = uint16_t(raw[data_offset[response_0304] + data_len[response_0304]]) | (uint16_t(raw[data_offset[response_0304] + data_len[response_0304] + 1]) << 8);
      if (computed_crc == remote_crc)  {
        frame_type = response_0304;
      }
    }
    if ((at<MAX_MESSAGE_SIZE)  && (frame_type==no_frame)) {
      return true; //not enough bytes
    }
  }

    // the response for write command mirrors the requests and data starts at offset 2 instead of 3 for read commands

  //  if (function_code == 0x5 || function_code == 0x06 || function_code == 0xF || function_code == 0x10) {
  //    data_offset = 2;
  //    data_len = 4;
  //  }
  else if ((function_code == 0x5 || function_code == 0x06 || function_code == 0xF || function_code == 0x10)){
    //check for command
    if (at > data_offset[command_05060F10]+data_len[command_05060F10]) {
      uint16_t computed_crc = crc16(raw, data_offset[command_05060F10] + data_len[command_05060F10]);
      uint16_t remote_crc = uint16_t(raw[data_offset[command_05060F10] + data_len[command_05060F10]]) | (uint16_t(raw[data_offset[command_05060F10] + data_len[command_05060F10] + 1]) << 8);
      if (computed_crc == remote_crc) {
        frame_type = command_05060F10;
      }
    }
      //check for response
    if ((at > data_offset[response_05060F10]+data_len[response_05060F10]) && (frame_type==no_frame)) { //write responses are 8 bytes)
      uint16_t computed_crc = crc16(raw, data_offset[response_05060F10] + data_len[response_05060F10]);
      uint16_t remote_crc = uint16_t(raw[data_offset[response_05060F10] + data_len[response_05060F10]]) | (uint16_t(raw[data_offset[response_05060F10] + data_len[response_05060F10] + 1]) << 8);
      if (computed_crc == remote_crc)  {
        frame_type = response_05060F10;
      }
    }
    if ((at<MAX_MESSAGE_SIZE)  && (frame_type==no_frame)){
      return true; //not enough bytes
    }
  } 
  else  if ((function_code & 0x80) == 0x80)  {   // Error ( function code msb indicates error )
    // response format:  Byte[0] = device address, Byte[1] function code | 0x80 , Byte[2] exception code, Byte[3-4] crc
    frame_type = error_80;
  }


  // Byte data_offset+len+1: CRC_HI (over all bytes)
  uint16_t computed_crc = crc16(raw, data_offset[frame_type] + data_len[frame_type]);
  uint16_t remote_crc = uint16_t(raw[data_offset[frame_type] + data_len[frame_type]]) | (uint16_t(raw[data_offset[frame_type] + data_len[frame_type] + 1]) << 8);

  if (computed_crc != remote_crc) {
    if (this->disable_crc_) {
      ESP_LOGD(TAG, "Modbus CRC Check failed, but ignored! %02X!=%02X", computed_crc, remote_crc);
    } else {
      ESP_LOGW(TAG, "Modbus CRC Check fail! %02X!=%02X", computed_crc, remote_crc);
      // std::string raw_bytes;
      // for (size_t i = 0; i < this->rx_buffer_.size(); i++) {
      //     char hex[4];
      //     snprintf(hex, sizeof(hex), "%02X ", this->rx_buffer_[i]);
      //     raw_bytes += hex;
      // }
      // if(this->role == ModbusRole::SERVER){
      //   ESP_LOGW(TAG, "SERVER Failed CRC msg:    %s", raw_bytes.c_str()); 
      // } else {
      //   ESP_LOGW(TAG, "  CLIENT Failed CRC msg:    %s", raw_bytes.c_str());         
      // }
      return false;
    }
  }
  

  uint16_t start_reg= uint16_t(raw[3]) | (uint16_t(raw[2]) << 8);
  uint16_t num_regs= uint16_t(raw[5]) | (uint16_t(raw[4]) << 8);
//  std::vector<uint8_t> data(this->rx_buffer_.begin() + data_offset, this->rx_buffer_.begin() + data_offset + data_len);
  std::vector<uint8_t> data(this->rx_buffer_.begin() + data_offset[frame_type], this->rx_buffer_.begin() + data_offset[frame_type] + data_len[frame_type]);
  
  // if (address == 0x0F ) {
  //   const char *frame_type_str = nullptr;
  //   switch (frame_type) {
  //     case no_frame:
  //       frame_type_str = "no_frame";
  //       break;
  //     case command_0304:
  //       frame_type_str = "command_0304";
  //       break;
  //     case response_0304:
  //       frame_type_str = "response_0304";
  //       break;
  //     case command_05060F10:
  //       frame_type_str = "command_05060F10";
  //       break;
  //     case response_05060F10:
  //       frame_type_str = "response_05060F10";
  //       break;
  //     case response_custom:
  //       frame_type_str = "response_custom";
  //       break;
  //     case error_80:
  //       frame_type_str = "error_80";
  //       break;
  //   }
  //   ESP_LOGW(TAG, "Found addr: 0x%02x function 0x%02x frame_type %s start_reg %x num_regs %d data size %d",address, function_code, frame_type_str,start_reg,num_regs,data.size());
  // }

  // std::string device_list;         //  expect this should have all the devices from .yaml
  // for (auto *device : this->devices_) {
  //   char hex[4];
  //   snprintf(hex, sizeof(hex), "%02X ", device->address_);
  //   device_list += hex;
  //   device_list += " ";
  //   device_list += device->get_disable_send() ? "DISABLE SEND" : "ENABLE SEND"; 
  //   switch (this->role) {
  //     case ModbusRole::CLIENT:
  //       device_list += " CLIENT";
  //       break;
  //     case ModbusRole::SERVER:
  //       device_list += " SERVER";
  //       break;
  //     case ModbusRole::MUTE_CLIENT:
  //       device_list += " MUTE_CLIENT";
  //       break;
  //   }
  //   device_list += " ";
  // }
  // ESP_LOGW(TAG, "Modbus device list:    %s", device_list.c_str());

  bool found = false;

  for (auto *device : this->devices_) {
    if (device->address_ == address) {
      // Is it an error response?
      if ((function_code & 0x80) == 0x80) {
        ESP_LOGD(TAG, "Modbus error function code: 0x%X exception: %d", function_code, raw[2]);
        if (waiting_for_response != 0) {
          device->on_modbus_error(function_code & 0x7F, raw[2]);
        } else {
          // Ignore modbus exception not related to a pending command
          ESP_LOGD(TAG, "Ignoring Modbus error - not expecting a response");
        }
      } else if (this->role == ModbusRole::SERVER && (function_code == 0x3 || function_code == 0x4)) {
        ESP_LOGW(TAG, "SERVER read registers for address %02X! device->get_disable_send() %d", address, device->get_disable_send());
        device->on_modbus_read_registers(function_code,start_reg,num_regs);
      } else {
        device->on_modbus_data(is_response[frame_type],address,function_code,start_reg,num_regs,remote_crc,data);
      }
      found = true;
    }
  }
  waiting_for_response = 0;

  if (!found) {
    ESP_LOGW(TAG, "Frame from unknown addr 0x%02X! ", address);
    // // Check if the address is 0x0F before logging the raw message bytes
    // if (address == 0x0F ) {
    //     std::string raw_bytes;
    //     for (size_t i = 0; i < this->rx_buffer_.size(); i++) {
    //         char hex[4];
    //         snprintf(hex, sizeof(hex), "%02X ", this->rx_buffer_[i]);
    //         raw_bytes += hex;
    //     }
    //     ESP_LOGW(TAG, "Batt1:    %s", raw_bytes.c_str()); 
    // }

  }

  // reset buffer
  ESP_LOGV(TAG, "Clearing buffer of %d bytes - parse succeeded", at);
  this->rx_buffer_.clear();
  return true;
}

void Modbus::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus:");
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  ESP_LOGCONFIG(TAG, "  Send Wait Time: %d ms", this->send_wait_time_);
  ESP_LOGCONFIG(TAG, "  CRC Disabled: %s", YESNO(this->disable_crc_));
}
float Modbus::get_setup_priority() const {
  // After UART bus
  return setup_priority::BUS - 1.0f;
}

void Modbus::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities,
                  uint8_t payload_len, const uint8_t *payload, bool disable_send) {
  static const size_t MAX_VALUES = 128;

  // Only check max number of registers for standard function codes
  // Some devices use non standard codes like 0x43
  if (number_of_entities > MAX_VALUES && function_code <= 0x10) {
    ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
    return;
  }

  std::vector<uint8_t> data;
  data.push_back(address);
  data.push_back(function_code);
  if (this->role == ModbusRole::CLIENT) {
    data.push_back(start_address >> 8);
    data.push_back(start_address >> 0);
    if (function_code != 0x5 && function_code != 0x6) {
      data.push_back(number_of_entities >> 8);
      data.push_back(number_of_entities >> 0);
    }
  }

  if (payload != nullptr) {
    if (this->role == ModbusRole::SERVER || function_code == 0xF || function_code == 0x10) {  // Write multiple
      data.push_back(payload_len);  // Byte count is required for write
    } else {
      payload_len = 2;  // Write single register or coil
    }
    for (int i = 0; i < payload_len; i++) {
      data.push_back(payload[i]);
    }
  }

  auto crc = crc16(data.data(), data.size());
  data.push_back(crc >> 0);
  data.push_back(crc >> 8);

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  this->write_array(data);
  this->flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = address;
  last_send_ = millis();
  ESP_LOGV(TAG, "Modbus write: %s", format_hex_pretty(data).c_str());
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload
void Modbus::send_raw(const std::vector<uint8_t> &payload, bool disable_send) {
  if (payload.empty()) {
    return;
  }

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  auto crc = crc16(payload.data(), payload.size());
  if (not disable_send)  {
    this->write_array(payload);
    this->write_byte(crc & 0xFF);
    this->write_byte((crc >> 8) & 0xFF);
    //this->flush();
  }
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);
  waiting_for_response = payload[0];
  ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
  last_send_ = millis();
}

}  // namespace modbus
}  // namespace esphome
