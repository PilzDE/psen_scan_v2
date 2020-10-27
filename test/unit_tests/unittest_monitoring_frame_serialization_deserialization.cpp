// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include <gtest/gtest.h>

#include "psen_scan_v2/diagnostics.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/scanner_ids.h"

#include "psen_scan_v2/istring_stream_builder.h"
#include "psen_scan_v2/monitoring_frame_serialization.h"
#include "psen_scan_v2/raw_data_array_conversion.h"
#include "psen_scan_v2/udp_frame_dumps.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
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
  std::array<ErrorLocation, 3> error_locations = { ErrorLocation(0, 0), ErrorLocation(5, 0), ErrorLocation(4, 7) };

  for (const auto& elem : error_locations)
  {
    ASSERT_NE(error_bits.at(elem.getByte()).at(elem.getBit()), Dc::UNUSED)
        << "The unused diagnostic bits are discarded during deserialization. You should use different test data for "
           "this test.";
  }

  MonitoringFrameMsg msg(TenthOfDegree(25),
                         TenthOfDegree(1),
                         456,
                         { 10, 20, 30, 40 },
                         { 15, 25, 35, 45 },
                         { MonitoringFrameDiagnosticMessage(ScannerId::MASTER, error_locations.at(0)),
                           MonitoringFrameDiagnosticMessage(ScannerId::MASTER, error_locations.at(1)),
                           MonitoringFrameDiagnosticMessage(ScannerId::SLAVE2, error_locations.at(2)) });

  auto raw = serialize(msg);

  auto deserialized_msg = deserializeMonitoringFrame(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(msg, deserialized_msg);
}

TEST(MonitoringFrameSerializationTest, shouldFailOnSerializeAndDeserializeFrameWithIntensityChannelBits)
{
  MonitoringFrameMsg msg(TenthOfDegree(25), TenthOfDegree(1), 1, { 0 }, { 70045 }, {});

  auto raw = serialize(msg);
  auto deserialized_msg = deserializeMonitoringFrame(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_FALSE(msg == deserialized_msg);
  EXPECT_EQ(deserialized_msg.intensities().at(0), 0b0011111111111111 & 70045);
}

TEST(MonitoringFrameSerializationDiagnosticMessagesTest, shouldSetCorrectBitInSerializedDiagnosticData)
{
  std::vector<MonitoringFrameDiagnosticMessage> diagnostic_data{ { ScannerId::MASTER, ErrorLocation(5, 3) } };
  auto diagnostic_data_serialized = serializeDiagnosticMessages(diagnostic_data);

  EXPECT_EQ(diagnostic_data_serialized.size(), RAW_DIAGNOSTIC_MESSAGE_LENGTH_IN_BYTES);
  EXPECT_EQ(diagnostic_data_serialized.at(RAW_DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES + 5), 0b1000);
}

TEST(MonitoringFrameDeserializationFieldHeaderTest, shouldGetIdAndLengthCorrectly)
{
  uint8_t id = 5;
  uint16_t length = 7;
  MonitoringFrameAdditionalFieldHeader header(id, length);
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

  std::unique_ptr<MonitoringFrameAdditionalFieldHeader> header_ptr;
  ASSERT_NO_THROW(header_ptr.reset(new MonitoringFrameAdditionalFieldHeader{ readFieldHeader(is, max_num_bytes) }););
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

  EXPECT_THROW(readFieldHeader(is, max_num_bytes);, raw_processing::StringStreamFailure);
}
class MonitoringFrameDeserializationTest : public ::testing::Test
{
protected:
  MonitoringFrameDeserializationTest()
  {
    with_intensities_raw_ = convertToMaxSizeRawData(with_intensities_.hex_dump);
  }

protected:
  MaxSizeRawData with_intensities_raw_;
  scanner_udp_datagram_hexdumps::WithIntensitiesAndDiagnostics with_intensities_;
};

TEST_F(MonitoringFrameDeserializationTest, shouldDeserializeMonitoringFrameCorrectly)
{
  MonitoringFrameMsg msg;
  ASSERT_NO_THROW(msg = deserializeMonitoringFrame(with_intensities_raw_, with_intensities_raw_.size()););
  EXPECT_EQ(msg, with_intensities_.expected_msg_);
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnWrongOpCode)
{
  with_intensities_raw_.at(4) += 1;
  EXPECT_NO_THROW(deserializeMonitoringFrame(with_intensities_raw_, with_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnInvalidWorkingMode)
{
  with_intensities_raw_.at(8) = 0x03;
  EXPECT_NO_THROW(deserializeMonitoringFrame(with_intensities_raw_, with_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnInvalidTransactionType)
{
  with_intensities_raw_.at(12) = 0x06;
  EXPECT_NO_THROW(deserializeMonitoringFrame(with_intensities_raw_, with_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnInvalidScannerId)
{
  with_intensities_raw_.at(16) = 0x04;
  EXPECT_NO_THROW(deserializeMonitoringFrame(with_intensities_raw_, with_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnUnknownFieldId)
{
  scanner_udp_datagram_hexdumps::WithUnknownFieldId with_unknown_field_id;
  const auto raw_frame_data = convertToMaxSizeRawData(with_unknown_field_id.hex_dump);
  const auto num_bytes = 2 * with_unknown_field_id.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = deserializeMonitoringFrame(raw_frame_data, num_bytes);, MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeFieldLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeFieldLength with_too_large_field_length;
  const auto raw_frame_data = convertToMaxSizeRawData(with_too_large_field_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_field_length.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = deserializeMonitoringFrame(raw_frame_data, num_bytes);, MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeScanCounterLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeScanCounterLength with_too_large_scan_counter_length;
  const auto raw_frame_data = convertToMaxSizeRawData(with_too_large_scan_counter_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_scan_counter_length.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = deserializeMonitoringFrame(raw_frame_data, num_bytes);
               , MonitoringFrameFormatErrorScanCounterUnexpectedSize);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
