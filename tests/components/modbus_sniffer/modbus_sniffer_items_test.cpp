#include <gtest/gtest.h>

#include <cstdint>
#include <span>
#include <vector>

#include "common.h"
#include "esphome/components/modbus/modbus.h"
#include "esphome/components/modbus_sniffer/modbus_sniffer.h"

namespace esphome::modbus_sniffer::testing {

namespace {

using modbus::testing::SnifferUART;

/// Stands in for a platform item. The sensor platform converts the raw word to a float and
/// publishes; what is under test here is which items are fed, with what, and when.
class RecordingItem : public SnifferItem {
 public:
  RecordingItem(uint8_t address, uint16_t reg, bool is_signed) : SnifferItem(address, reg), is_signed_(is_signed) {}

  void parse_and_publish(uint16_t raw) override {
    this->published.push_back(this->is_signed_ ? static_cast<float>(static_cast<int16_t>(raw))
                                               : static_cast<float>(raw));
  }

  std::vector<float> published;

 private:
  bool is_signed_;
};

/// Hub plus the component the sensors hang off, wired as codegen would.
class SensorFixture {
 public:
  SensorFixture() {
    hub.set_uart_parent(&uart);
    sniffer.set_parent(&hub);
  }

  /// setup() is what subscribes, so it must run after the sensors are added.
  void start() { sniffer.setup(); }

  void run(size_t passes = 4) {
    for (size_t i = 0; i < passes; i++)
      hub.loop();
  }

  modbus::ModbusSnifferHub hub;
  SnifferUART uart;
  ModbusSniffer sniffer;
};

constexpr uint8_t FC_READ_HOLDING = static_cast<uint8_t>(modbus::FunctionCode::READ_HOLDING_REGISTERS);
constexpr uint8_t FC_READ_INPUT = static_cast<uint8_t>(modbus::FunctionCode::READ_INPUT_REGISTERS);

std::vector<uint8_t> read_request(uint8_t function_code, uint16_t start, uint16_t count) {
  return {function_code, static_cast<uint8_t>(start >> 8), static_cast<uint8_t>(start & 0xFF),
          static_cast<uint8_t>(count >> 8), static_cast<uint8_t>(count & 0xFF)};
}

std::vector<uint8_t> read_response(uint8_t function_code, std::span<const uint16_t> registers) {
  std::vector<uint8_t> pdu{function_code, static_cast<uint8_t>(registers.size() * 2)};
  for (uint16_t value : registers) {
    pdu.push_back(static_cast<uint8_t>(value >> 8));
    pdu.push_back(static_cast<uint8_t>(value & 0xFF));
  }
  return pdu;
}

}  // namespace

// A register inside the block a request covered is published when the reply goes past.
TEST(ModbusSnifferItems, PublishesARegisterFromAMatchingRead) {
  SensorFixture f;
  RecordingItem bus_voltage{static_cast<uint8_t>(0x0F), 202, false};
  f.sniffer.add_item(&bus_voltage);
  f.start();

  const uint16_t registers[] = {0x0FC8, 0x1004};  // 201, 202
  f.uart.inject_frame(0x0F, read_request(FC_READ_HOLDING, 201, 2));
  f.uart.inject_frame(0x0F, read_response(FC_READ_HOLDING, registers));
  f.run();

  ASSERT_EQ(bus_voltage.published.size(), 1u);
  EXPECT_FLOAT_EQ(bus_voltage.published[0], 0x1004);
}

// S_WORD reinterprets the same 16 bits: a discharging battery reads negative, not ~65000.
TEST(ModbusSnifferItems, SignedValuesAreReinterpretedNotRescaled) {
  SensorFixture f;
  RecordingItem signed_power{static_cast<uint8_t>(0x0F), 203, true};
  f.sniffer.add_item(&signed_power);
  RecordingItem unsigned_power{static_cast<uint8_t>(0x0F), 203, false};
  f.sniffer.add_item(&unsigned_power);
  f.start();

  const uint16_t registers[] = {0xF7B4};  // -2124 as int16_t
  f.uart.inject_frame(0x0F, read_request(FC_READ_HOLDING, 203, 1));
  f.uart.inject_frame(0x0F, read_response(FC_READ_HOLDING, registers));
  f.run();

  ASSERT_EQ(signed_power.published.size(), 1u);
  EXPECT_FLOAT_EQ(signed_power.published[0], -2124.0f);
  ASSERT_EQ(unsigned_power.published.size(), 1u);
  EXPECT_FLOAT_EQ(unsigned_power.published[0], 63412.0f);
}

// Two devices on one bus. A sensor only answers to its own address, or a meter register would be
// published from a battery reply that happens to cover the same number.
TEST(ModbusSnifferItems, AddressSelectsBetweenDevicesOnTheSameBus) {
  SensorFixture f;
  RecordingItem battery{static_cast<uint8_t>(0x0F), 40018, false};
  f.sniffer.add_item(&battery);
  RecordingItem meter{static_cast<uint8_t>(0x02), 40018, true};
  f.sniffer.add_item(&meter);
  f.start();

  const uint16_t registers[] = {0xFFB0};  // -80
  f.uart.inject_frame(0x02, read_request(FC_READ_INPUT, 40018, 1));
  f.uart.inject_frame(0x02, read_response(FC_READ_INPUT, registers));
  f.run();

  ASSERT_EQ(meter.published.size(), 1u);
  EXPECT_FLOAT_EQ(meter.published[0], -80.0f);
  EXPECT_TRUE(battery.published.empty());
}

// A register outside the block this exchange covered is not published from it.
TEST(ModbusSnifferItems, IgnoresRegistersTheExchangeDidNotCover) {
  SensorFixture f;
  RecordingItem before_block{static_cast<uint8_t>(0x0F), 200, false};
  f.sniffer.add_item(&before_block);
  RecordingItem after_block{static_cast<uint8_t>(0x0F), 210, false};
  f.sniffer.add_item(&after_block);
  RecordingItem inside{static_cast<uint8_t>(0x0F), 202, false};
  f.sniffer.add_item(&inside);
  f.start();

  const uint16_t registers[] = {1, 2, 3};  // 201..203
  f.uart.inject_frame(0x0F, read_request(FC_READ_HOLDING, 201, 3));
  f.uart.inject_frame(0x0F, read_response(FC_READ_HOLDING, registers));
  f.run();

  EXPECT_TRUE(before_block.published.empty());
  EXPECT_TRUE(after_block.published.empty());
  EXPECT_EQ(inside.published.size(), 1u);
}

// Input registers (0x04) decode exactly as holding registers (0x03) do.
TEST(ModbusSnifferItems, PublishesFromInputRegisterReads) {
  SensorFixture f;
  RecordingItem s{static_cast<uint8_t>(0x02), 40003, false};
  f.sniffer.add_item(&s);
  f.start();

  const uint16_t registers[] = {0xBEEF};
  f.uart.inject_frame(0x02, read_request(FC_READ_INPUT, 40003, 1));
  f.uart.inject_frame(0x02, read_response(FC_READ_INPUT, registers));
  f.run();

  ASSERT_EQ(s.published.size(), 1u);
  EXPECT_FLOAT_EQ(s.published[0], 0xBEEF);
}

// An exception reply carries an error code where the registers would be; publishing from it would
// report the error as a measurement.
TEST(ModbusSnifferItems, DoesNotPublishFromAnExceptionReply) {
  SensorFixture f;
  RecordingItem s{static_cast<uint8_t>(0x0F), 202, false};
  f.sniffer.add_item(&s);
  f.start();

  f.uart.inject_frame(0x0F, read_request(FC_READ_HOLDING, 202, 1));
  f.uart.inject_frame(0x0F, std::vector<uint8_t>{FC_READ_HOLDING | 0x80, 0x02});  // ILLEGAL_DATA_ADDRESS
  f.run();

  EXPECT_TRUE(s.published.empty());
}

// A write echoes its own register address where a read puts data, so it is not a register read a
// sensor can index.
TEST(ModbusSnifferItems, DoesNotPublishFromAWriteExchange) {
  SensorFixture f;
  RecordingItem s{static_cast<uint8_t>(0x0F), 1101, false};
  f.sniffer.add_item(&s);
  f.start();

  const std::vector<uint8_t> write{static_cast<uint8_t>(modbus::FunctionCode::WRITE_SINGLE_REGISTER), 0x04, 0x4D, 0x00,
                                   0x01};
  f.uart.inject_frame(0x0F, write);
  f.uart.inject_frame(0x0F, write);
  f.run();

  EXPECT_TRUE(s.published.empty());
}

// One exchange feeds every sensor it covers, so a block read publishes them all in one pass.
TEST(ModbusSnifferItems, OneExchangeFeedsEverySensorItCovers) {
  SensorFixture f;
  RecordingItem a{static_cast<uint8_t>(0x0F), 201, false};
  f.sniffer.add_item(&a);
  RecordingItem b{static_cast<uint8_t>(0x0F), 203, true};
  f.sniffer.add_item(&b);
  RecordingItem c{static_cast<uint8_t>(0x0F), 206, false};
  f.sniffer.add_item(&c);
  f.start();

  const uint16_t registers[] = {11, 12, 0xFFFF, 14, 15, 16};  // 201..206
  f.uart.inject_frame(0x0F, read_request(FC_READ_HOLDING, 201, 6));
  f.uart.inject_frame(0x0F, read_response(FC_READ_HOLDING, registers));
  f.run();

  ASSERT_EQ(a.published.size(), 1u);
  EXPECT_FLOAT_EQ(a.published[0], 11.0f);
  ASSERT_EQ(b.published.size(), 1u);
  EXPECT_FLOAT_EQ(b.published[0], -1.0f);  // signed
  ASSERT_EQ(c.published.size(), 1u);
  EXPECT_FLOAT_EQ(c.published[0], 16.0f);
}

}  // namespace esphome::modbus_sniffer::testing
