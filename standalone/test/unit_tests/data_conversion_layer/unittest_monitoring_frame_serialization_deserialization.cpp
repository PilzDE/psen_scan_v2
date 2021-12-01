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
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/io_state.h"

#include "psen_scan_v2_standalone/data_conversion_layer/istring_stream_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_serialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"
#include "psen_scan_v2_standalone/communication_layer/udp_frame_dumps.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

using namespace psen_scan_v2_standalone;
using namespace data_conversion_layer;

namespace psen_scan_v2_standalone_test
{
TEST(MonitoringFrameSerializationTest, shouldSerializeHexdumpFrameCorrectly)
{
  scanner_udp_datagram_hexdumps::WithIntensitiesAndDiagnostics with_intensities;
  auto serialized_monitoring_frame_message = serialize(with_intensities.expected_msg_);

  EXPECT_EQ(with_intensities.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < with_intensities.hex_dump.size(); i++)
  {
    uint8_t expected_byte = scanner_udp_datagram_hexdumps::clearIntensityChannelBits(
        i,
        with_intensities.intensities_offset,
        2 * with_intensities.expected_msg_.intensities().size(),
        with_intensities.hex_dump.at(i));
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), expected_byte) << " index " << i;
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

  auto msg = monitoring_frame::MessageBuilder()
                 .fromTheta(util::TenthOfDegree(25))
                 .resolution(util::TenthOfDegree(1))
                 .scanCounter(456)
                 .activeZoneset(2)
                 .measurements({ 10, 20, std::numeric_limits<double>::infinity(), 40 })
                 .intensities({ 15, 25, 35, 45 })
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

using namespace monitoring_frame::io;
TEST_F(MonitoringFrameDeserializationTest, shouldCreateCorrectPhysicalInputField)
{
  auto raw = convertToRawData(std::array<uint8_t, 10>{ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00 });
  std::stringstream ss;
  ss.write(raw.data(), 10);
  auto input = deserializePinField(ss, 10, createInputPinState);
  std::vector<PinState> expected_states{
    PinState(48, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_1), true),
    PinState(49, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_2), false),
    PinState(50, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_3), true),
    PinState(51, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_4), false),
    PinState(52, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_5), true),
    PinState(53, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_6), false),
    PinState(54, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_7), true),
    PinState(55, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::zone_sw_8), false),
    PinState(56, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::reset), true),
    // unused bit
    PinState(58, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::restart_1), true),
    PinState(59, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_en_1), false),
    PinState(60, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_11), true),
    PinState(61, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_12), false),
    PinState(62, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::override_11), true),
    PinState(63, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::override_12), false),
    PinState(64, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::edm_1), true),
    PinState(65, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::restart_2), false),
    PinState(66, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_en_2), true),
    PinState(67, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_21), false),
    PinState(68, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_22), true),
    PinState(69, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::override_21), false),
    PinState(70, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::override_22), true),
    PinState(71, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::edm_2), false),
    PinState(72, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::restart_3), false),
    PinState(73, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_en_3), false),
    PinState(74, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_31), false),
    PinState(75, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::muting_32), false),
    PinState(76, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::override_31), false),
    PinState(77, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::override_32), false),
    PinState(78, PHYSICAL_INPUT_BIT_TO_NAME.at(PhysicalInputType::edm_3), false)
    // unused bit
  };

  EXPECT_CONTAINER_UNORDERED_EQ(input, expected_states);
}

TEST_F(MonitoringFrameDeserializationTest, shouldCreateCorrectLogicalInputField)
{
  auto raw = convertToRawData(std::array<uint8_t, 8>{ 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00 });
  std::stringstream ss;
  ss.write(raw.data(), 8);
  auto input = deserializePinField(ss, 8, createLogicalPinState);
  std::vector<PinState> expected_states{};

  uint32_t i = 0;
  for (; i < 56; i += 2)
  {
    expected_states.push_back(PinState(i, "", true));
    expected_states.push_back(PinState(i + 1, "", false));
  }
  for (; i < 64; i++)
  {
    expected_states.push_back(PinState(i, "", false));
  }

  EXPECT_CONTAINER_UNORDERED_EQ(input, expected_states);
}

TEST_F(MonitoringFrameDeserializationTest, shouldCreateCorrectOutputField)
{
  auto raw = convertToRawData(std::array<uint8_t, 4>{ 0x55, 0x55, 0x55, 0x00 });
  std::stringstream ss;
  ss.write(raw.data(), 4);
  auto output = deserializePinField(ss, 4, createOutputPinState);
  std::vector<PinState> expected_states{
    PinState(0, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1), true),
    PinState(1, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_lock), false),
    PinState(2, OUTPUT_BIT_TO_NAME.at(OutputType::ossd2), true),
    PinState(3, OUTPUT_BIT_TO_NAME.at(OutputType::ossd2_lock), false),
    PinState(4, OUTPUT_BIT_TO_NAME.at(OutputType::ossd3), true),
    PinState(5, OUTPUT_BIT_TO_NAME.at(OutputType::ossd3_lock), false),
    PinState(6, OUTPUT_BIT_TO_NAME.at(OutputType::warn1), true),
    PinState(7, OUTPUT_BIT_TO_NAME.at(OutputType::warn2), false),
    PinState(8, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_m), true),
    PinState(9, OUTPUT_BIT_TO_NAME.at(OutputType::ossd2_m), false),
    PinState(10, OUTPUT_BIT_TO_NAME.at(OutputType::ossd3_m), true),
    PinState(11, OUTPUT_BIT_TO_NAME.at(OutputType::warn1_m), false),
    PinState(12, OUTPUT_BIT_TO_NAME.at(OutputType::warn2_m), true),
    PinState(13, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_slv1), false),
    PinState(14, OUTPUT_BIT_TO_NAME.at(OutputType::ossd2_slv1), true),
    PinState(15, OUTPUT_BIT_TO_NAME.at(OutputType::ossd3_slv1), false),
    PinState(16, OUTPUT_BIT_TO_NAME.at(OutputType::warn1_slv1), true),
    PinState(17, OUTPUT_BIT_TO_NAME.at(OutputType::warn2_slv1), false),
    PinState(18, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_slv2), true),
    PinState(19, OUTPUT_BIT_TO_NAME.at(OutputType::ossd2_slv2), false),
    PinState(20, OUTPUT_BIT_TO_NAME.at(OutputType::ossd3_slv2), true),
    PinState(21, OUTPUT_BIT_TO_NAME.at(OutputType::warn1_slv2), false),
    PinState(22, OUTPUT_BIT_TO_NAME.at(OutputType::warn2_slv2), true),
    PinState(23, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_slv3), false),
    PinState(24, OUTPUT_BIT_TO_NAME.at(OutputType::ossd2_slv3), false),
    PinState(25, OUTPUT_BIT_TO_NAME.at(OutputType::ossd3_slv3), false),
    PinState(26, OUTPUT_BIT_TO_NAME.at(OutputType::warn1_slv3), false),
    PinState(27, OUTPUT_BIT_TO_NAME.at(OutputType::warn2_slv3), false),
    PinState(28, OUTPUT_BIT_TO_NAME.at(OutputType::ossd1_refpts), false),
    // unused bit
    // unused bit
    // unused bit
  };

  EXPECT_CONTAINER_UNORDERED_EQ(output, expected_states);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
