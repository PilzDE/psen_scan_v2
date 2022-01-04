// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <vector>
#include <array>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/io_state.h"

#include "psen_scan_v2_standalone/data_conversion_layer/istring_stream_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_serialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data_helper.h"
#include "psen_scan_v2_standalone/communication_layer/udp_frame_dumps.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

using namespace psen_scan_v2_standalone;
using namespace data_conversion_layer;

template <typename T>
void printDump(const T& data)
{
  size_t id = 0;
  size_t len = 0;
  size_t next_offset = 21;
  std::cout << "Const part" << std::endl;
  for (size_t i = 0; i < data.size(); i++)
  {
    if (i == next_offset)
    {
      id = data.at(i);
      if (id != 0x09)
      {
        len = (((unsigned char)data.at(i + 2)) << 8) | ((unsigned char)data.at(i + 1));
        next_offset = i + len + 3 - 1;
      }
    }
    std::cout << fmt::format("\x1B[{}m{:#04x}\033[0m", 30 + id, (uint8_t)data.at(i)) << " ";
    if (i % 10 == 0)
    {
      std::cout << i << "    ID " << id << " len: " << len;
      std::cout << std::endl;
    }
  }
}

namespace psen_scan_v2_standalone_test
{
namespace io = monitoring_frame::io;

TEST(MonitoringFrameSerializationTest, shouldSerializeHexdumpFrameCorrectly)
{
  scanner_udp_datagram_hexdumps::WithIntensitiesAndDiagnostics with_intensities;
  auto serialized_monitoring_frame_message = serialize(with_intensities.expected_msg_);
  printDump(serialized_monitoring_frame_message);
  printDump(with_intensities.hex_dump);

  ASSERT_EQ(with_intensities.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < with_intensities.hex_dump.size(); i++)
  {
    // Intensities are a special case since here only the last 14 bytes are important since
    // the first two are reserved for the channel which is not checked
    if (i > with_intensities.intensities_offset &&
        i < with_intensities.intensities_offset + 2 * with_intensities.expected_msg_.intensities().size())
    {
      // The following line makes sure we compare the proper tuples. E.g. if the offset is 650
      // we want to compare (650, 651), (652, 653), ....
      if (i % 2 == with_intensities.intensities_offset % 2)
      {
        uint16_t raw_value_expected = scanner_udp_datagram_hexdumps::convertHexdumpBytesToUint16_t(
            with_intensities.hex_dump.at(i + 1), with_intensities.hex_dump.at(i));
        uint16_t raw_value_actual = scanner_udp_datagram_hexdumps::convertHexdumpBytesToUint16_t(
            serialized_monitoring_frame_message.at(i + 1), serialized_monitoring_frame_message.at(i));

        ASSERT_EQ(0b0011111111111111 & raw_value_expected, 0b0011111111111111 & raw_value_actual)
            << " index " << i << " hexdump value: " << fmt::format("{:#04x}", (uint16_t)raw_value_expected)
            << " actual_value: " << fmt::format("{:#04x}", (uint16_t)raw_value_actual);
      }
    }
    // Compare all non intensity values directly
    else
    {
      ASSERT_EQ((uint8_t)with_intensities.hex_dump.at(i), (uint8_t)serialized_monitoring_frame_message.at(i))
          << " index " << i << " hexdump value: " << fmt::format("{:#04x}", (uint8_t)with_intensities.hex_dump.at(i))
          << " actual_value: " << fmt::format("{:#04x}", (uint8_t)serialized_monitoring_frame_message.at(i));
    }
  }
}

TEST(MonitoringFrameSerializationTest, shouldSerializeHexdumpFrameWithoutMeasurementsAndIntensitiesCorrectly)
{
  scanner_udp_datagram_hexdumps::WithoutMeasurementsAndIntensities without_measurements_and_intensities;
  auto serialized_monitoring_frame_message = serialize(without_measurements_and_intensities.expected_msg_);

  EXPECT_EQ(without_measurements_and_intensities.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < without_measurements_and_intensities.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), without_measurements_and_intensities.hex_dump.at(i))
        << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, shouldSerializeAndDeserializeFrameConsistently)
{
  std::array<monitoring_frame::diagnostic::ErrorLocation, 3> error_locations = {
    monitoring_frame::diagnostic::ErrorLocation(0, 0),
    monitoring_frame::diagnostic::ErrorLocation(5, 0),
    monitoring_frame::diagnostic::ErrorLocation(4, 7)
  };

  for (const auto& elem : error_locations)
  {
    ASSERT_NE(monitoring_frame::diagnostic::ERROR_BITS.at(elem.getByte()).at(elem.getBit()),
              monitoring_frame::diagnostic::ErrorType::unused)
        << "The unused diagnostic bits are discarded during deserialization. You should use different test data for "
           "this test.";
  }

  auto pin_data = createCompleteIOPinData();
  setInputPin(pin_data, LogicalInputType::muting_1_a);
  setOutputPin(pin_data, OutputType::safe_1_int);

  auto msg = monitoring_frame::MessageBuilder()
                 .fromTheta(util::TenthOfDegree(25))
                 .resolution(util::TenthOfDegree(1))
                 .scanCounter(456)
                 .activeZoneset(2)
                 .measurements({ 10, 20, std::numeric_limits<double>::infinity(), 40 })
                 .intensities({ 15, 25, 35, 45 })
                 .iOPinData(pin_data)
                 .diagnosticMessages(
                     { monitoring_frame::diagnostic::Message(configuration::ScannerId::master, error_locations.at(0)),
                       monitoring_frame::diagnostic::Message(configuration::ScannerId::master, error_locations.at(1)),
                       monitoring_frame::diagnostic::Message(configuration::ScannerId::slave2, error_locations.at(2)) })
                 .build();

  auto raw = serialize(msg);
  auto deserialized_msg = monitoring_frame::deserialize(convertToRawData(raw), raw.size());

  EXPECT_THAT(deserialized_msg, MonitoringFrameEq(msg));
}

TEST(MonitoringFrameSerializationTest, shouldFailOnSerializeAndDeserializeFrameWithIntensityChannelBits)
{
  auto msg = data_conversion_layer::monitoring_frame::MessageBuilder()
                 .fromTheta(util::TenthOfDegree(25))
                 .resolution(util::TenthOfDegree(1))
                 .scanCounter(1)
                 .activeZoneset(0)
                 .measurements({ 0 })
                 .intensities({ 70045 })
                 .diagnosticMessages({})
                 .build();

  auto raw = serialize(msg);
  auto deserialized_msg = monitoring_frame::deserialize(convertToRawData(raw), raw.size());

  const uint32_t intensity_channel_bit_mask = 0b111111111100000000000000;
  ASSERT_NE(static_cast<uint32_t>(msg.intensities().at(0)) & intensity_channel_bit_mask, 0u);
  EXPECT_EQ(static_cast<uint32_t>(deserialized_msg.intensities().at(0)) & intensity_channel_bit_mask, 0u);
}

TEST(MonitoringFrameSerializationDiagnosticMessagesTest, shouldSetCorrectBitInSerializedDiagnosticData)
{
  std::vector<monitoring_frame::diagnostic::Message> diagnostic_data{
    { configuration::ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3) }
  };
  auto diagnostic_data_serialized = monitoring_frame::diagnostic::serialize(diagnostic_data);

  EXPECT_EQ(diagnostic_data_serialized.size(), monitoring_frame::diagnostic::RAW_CHUNK_LENGTH_IN_BYTES);
  EXPECT_EQ(diagnostic_data_serialized.at(monitoring_frame::diagnostic::RAW_CHUNK_UNUSED_OFFSET_IN_BYTES + 5), 0b1000);
}

TEST(MonitoringFrameDeserializationFieldHeaderTest, shouldGetIdAndLengthCorrectly)
{
  uint8_t id = 5;
  uint16_t length = 7;
  monitoring_frame::AdditionalFieldHeader header(id, length);
  EXPECT_EQ(id, header.id());
  EXPECT_EQ(length, header.length());
}

TEST(MonitoringFrameDeserializationFieldHeaderTest, shouldDeserializeMonitoringFrameAdditionalFieldHeaderCorrectly)
{
  uint8_t id = 5;
  uint16_t length = 7;
  uint16_t expected_length = length - 1;
  uint16_t max_num_bytes = 9;

  IStringStreamBuilder builder;
  builder.add(id);
  builder.add(length);
  std::istringstream is{ builder.get() };

  std::unique_ptr<monitoring_frame::AdditionalFieldHeader> header_ptr;
  ASSERT_NO_THROW(header_ptr.reset(
      new monitoring_frame::AdditionalFieldHeader{ monitoring_frame::readAdditionalField(is, max_num_bytes) }););
  EXPECT_EQ(id, header_ptr->id());
  EXPECT_EQ(expected_length, header_ptr->length());
}

TEST(MonitoringFrameDeserializationFieldHeaderTest, shouldFailOnReadHeaderWhichIsTooShort)
{
  uint16_t too_short_header;
  uint16_t max_num_bytes = 2;

  IStringStreamBuilder builder;
  builder.add(too_short_header);
  std::istringstream is{ builder.get() };

  EXPECT_THROW(monitoring_frame::readAdditionalField(is, max_num_bytes);, raw_processing::StringStreamFailure);
}
class MonitoringFrameDeserializationTest : public ::testing::Test
{
protected:
  MonitoringFrameDeserializationTest()
  {
    with_intensities_raw_ = convertToRawData(with_intensities_.hex_dump);
  }

protected:
  RawData with_intensities_raw_;
  scanner_udp_datagram_hexdumps::WithIntensitiesAndDiagnostics with_intensities_;
};

TEST_F(MonitoringFrameDeserializationTest, shouldDeserializeMonitoringFrameCorrectly)
{
  monitoring_frame::Message msg;
  ASSERT_NO_THROW(msg = monitoring_frame::deserialize(with_intensities_raw_, with_intensities_raw_.size()););
  EXPECT_THAT(msg, MonitoringFrameEq(with_intensities_.expected_msg_));
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnUnknownFieldId)
{
  scanner_udp_datagram_hexdumps::WithUnknownFieldId with_unknown_field_id;
  const auto raw_frame_data = convertToRawData(with_unknown_field_id.hex_dump);
  const auto num_bytes = 2 * with_unknown_field_id.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);, monitoring_frame::DecodingFailure);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeMonitoringLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeFieldLength with_too_large_field_length;
  const auto raw_frame_data = convertToRawData(with_too_large_field_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_field_length.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);, monitoring_frame::DecodingFailure);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeIntensityLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeIntensityLength with_too_large_field_length;
  const auto raw_frame_data = convertToRawData(with_too_large_field_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_field_length.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);, monitoring_frame::DecodingFailure);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnMissingEndOfFrame)
{
  scanner_udp_datagram_hexdumps::WithNoEnd with_no_end_of_frame;
  const auto raw_frame_data = convertToRawData(with_no_end_of_frame.hex_dump);
  const auto num_bytes = 2 * with_no_end_of_frame.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);, monitoring_frame::DecodingFailure);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeScanCounterLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeScanCounterLength with_too_large_scan_counter_length;
  const auto raw_frame_data = convertToRawData(with_too_large_scan_counter_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_scan_counter_length.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);
               , monitoring_frame::ScanCounterUnexpectedSize);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowZoneSetUnexpectedSizeErrorOnTooLargeZoneSetLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeActiveZoneSetLength with_too_large_active_zone_set_length;
  const auto raw_frame_data = convertToRawData(with_too_large_active_zone_set_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_active_zone_set_length.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);
               , monitoring_frame::ZoneSetUnexpectedSize);
}

TEST_F(MonitoringFrameDeserializationTest, shouldCreateCorrectLogicalInputField)
{
  auto raw = convertToRawData(std::array<uint8_t, 8>{ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00 });
  std::stringstream ss;
  ss.write(raw.data(), 8);
  auto input = deserializePinField(ss, 8, LOGICAL_INPUT_BITS, LOGICAL_INPUT_BIT_TO_NAME);

  std::vector<PinState> expected_states{
    PinState(0, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_0), true),
    PinState(1, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_1), false),
    PinState(2, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_2), true),
    PinState(3, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_3), false),
    PinState(4, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_4), true),
    PinState(5, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_5), false),
    PinState(6, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_6), true),
    PinState(7, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_bit_7), false),

    // unused bytes

    PinState(32, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::muting_1_a), true),
    PinState(33, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::muting_2_a), false),
    // unused bit
    PinState(35, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::overr_1_a), false),
    PinState(36, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::overr_2_a), true),
    // unused bit
    PinState(38, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_1), true),
    PinState(39, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_2), false),

    PinState(40, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_3), true),
    PinState(41, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_4), false),
    PinState(42, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_5), true),
    PinState(43, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_6), false),
    PinState(44, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_7), true),
    PinState(45, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::zone_sw_8), false),
    PinState(46, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::reset_a), true),
    // unused bit

    PinState(48, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::restart_1_a), true),
    PinState(49, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::mut_en_1_a), false),
    PinState(50, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::cor_seq_mut_1), true),
    PinState(51, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::cor_seq_or_1), false),
    // unused bit
    PinState(53, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::restart_2_a), false),
    PinState(54, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::mut_en_2_a), true),
    PinState(55, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::cor_seq_mut_2), false),

    PinState(56, LOGICAL_INPUT_BIT_TO_NAME.at(LogicalInputType::cor_seq_or_2), false),
    // unused bits
  };

  EXPECT_CONTAINER_UNORDERED_EQ(input, expected_states);
}

TEST_F(MonitoringFrameDeserializationTest, shouldCreateCorrectOutputField)
{
  auto raw = convertToRawData(std::array<uint8_t, 4>{ 0x55, 0x55, 0x55, 0x00 });
  std::stringstream ss;
  ss.write(raw.data(), 4);
  auto output = deserializePinField(ss, 4, OUTPUT_BITS, OUTPUT_BIT_TO_NAME);

  std::vector<PinState> expected_states{
    PinState(0, OUTPUT_BIT_TO_NAME.at(OutputType::safe_1_int), true),
    PinState(1, OUTPUT_BIT_TO_NAME.at(OutputType::int_lock_1), false),
    PinState(2, OUTPUT_BIT_TO_NAME.at(OutputType::safe_2_int), true),
    PinState(3, OUTPUT_BIT_TO_NAME.at(OutputType::int_lock_2), false),
    PinState(4, OUTPUT_BIT_TO_NAME.at(OutputType::safe_3_int), true),
    // unused bit
    PinState(6, OUTPUT_BIT_TO_NAME.at(OutputType::warn_1_int), true),
    PinState(7, OUTPUT_BIT_TO_NAME.at(OutputType::warn_2_int), false),

    // unused bytes

    PinState(28, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_refpts), false),
    // unused bits
  };

  EXPECT_CONTAINER_UNORDERED_EQ(output, expected_states);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
