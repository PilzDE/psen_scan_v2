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

#include <fmt/ostream.h>

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
TEST(MonitoringFrameSerializationTest, shouldSerializeFrameWithoutIntensitiesCorrectly)
{
  scanner_udp_datagram_hexdumps::WithoutIntensities without_intensities;
  DynamicSizeRawData serialized_monitoring_frame_message =
      monitoring_frame::serialize(without_intensities.expected_msg_);

  EXPECT_EQ(without_intensities.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < without_intensities.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), without_intensities.hex_dump.at(i)) << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, shouldSerializeFrameWithDiagnosticsCorrectly)
{
  scanner_udp_datagram_hexdumps::WithDiagnostics with_diagnostics;
  DynamicSizeRawData serialized_monitoring_frame_message = serialize(with_diagnostics.expected_msg_);

  EXPECT_EQ(with_diagnostics.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < with_diagnostics.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), with_diagnostics.hex_dump.at(i)) << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, shouldSerializeFrameWithoutMeasurementsAndIntensitiesCorrectly)
{
  scanner_udp_datagram_hexdumps::WithoutMeasurementsAndIntensities without_measurements_and_intensities;
  DynamicSizeRawData serialized_monitoring_frame_message =
      serialize(without_measurements_and_intensities.expected_msg_);

  EXPECT_EQ(without_measurements_and_intensities.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < without_measurements_and_intensities.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), without_measurements_and_intensities.hex_dump.at(i))
        << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, shouldSerializeAndDeserializeFrameWithoutIntensitiesConsistently)
{
  scanner_udp_datagram_hexdumps::WithoutIntensities without_intensities;
  DynamicSizeRawData raw = serialize(without_intensities.expected_msg_);

  monitoring_frame::Message deserialized_msg = monitoring_frame::deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(without_intensities.expected_msg_, deserialized_msg);
}

TEST(MonitoringFrameSerializationTest, shouldSerializeAndDeserializeFrameWithDiagnosticsConsistently)
{
  scanner_udp_datagram_hexdumps::WithDiagnostics with_diagnostics;
  DynamicSizeRawData raw = monitoring_frame::serialize(with_diagnostics.expected_msg_);

  monitoring_frame::Message deserialized_msg = monitoring_frame::deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(deserialized_msg, with_diagnostics.expected_msg_);
}

TEST(MonitoringFrameSerializationTest, shouldSerializeAndDeserializeSelfConstructedFrameWithDiagnosticsConsistently)
{
  std::array<monitoring_frame::ErrorLocation, 3> error_locations = { monitoring_frame::ErrorLocation(0, 0),
                                                                     monitoring_frame::ErrorLocation(5, 0),
                                                                     monitoring_frame::ErrorLocation(4, 7) };

  for (const auto& elem : error_locations)
  {
    ASSERT_NE(monitoring_frame::error_bits.at(elem.getByte()).at(elem.getBit()), monitoring_frame::Dc::UNUSED)
        << "The unused diagnostic bits are discarded during deserialization. You should use different test data for "
           "this test.";
  }

  monitoring_frame::Message msg(TenthOfDegree(25),
                                TenthOfDegree(1),
                                456,
                                { 10, 20, 30, 40 },
                                { monitoring_frame::DiagnosticMessage(ScannerId::MASTER, error_locations.at(0)),
                                  monitoring_frame::DiagnosticMessage(ScannerId::MASTER, error_locations.at(1)),
                                  monitoring_frame::DiagnosticMessage(ScannerId::SLAVE2, error_locations.at(2)) });

  DynamicSizeRawData raw = serialize(msg);

  monitoring_frame::Message deserialized_msg = monitoring_frame::deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(msg, deserialized_msg);
}

TEST(MonitoringFrameSerializationDiagnosticMessagesTest, shouldSetCorrectBitInSerializedDiagnosticData)
{
  std::vector<monitoring_frame::DiagnosticMessage> diagnostic_data{ { ScannerId::MASTER,
                                                                      monitoring_frame::ErrorLocation(5, 3) } };
  auto diagnostic_data_serialized = monitoring_frame::serializeDiagnosticMessages(diagnostic_data);

  EXPECT_EQ(diagnostic_data_serialized.size(), monitoring_frame::RAW_DIAGNOSTIC_MESSAGE_LENGTH_IN_BYTES);
  EXPECT_EQ(diagnostic_data_serialized.at(monitoring_frame::RAW_DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES + 5), 0b1000);
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
      new monitoring_frame::AdditionalFieldHeader{ monitoring_frame::readFieldHeader(is, max_num_bytes) }););
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

  EXPECT_THROW(monitoring_frame::readFieldHeader(is, max_num_bytes);, raw_processing::StringStreamFailure);
}
class MonitoringFrameDeserializationTest : public ::testing::Test
{
protected:
  MonitoringFrameDeserializationTest()
  {
    without_intensities_raw_ = convertToMaxSizeRawData(without_intensities_.hex_dump);
    with_diagnostics_raw_ = convertToMaxSizeRawData(with_diagnostics_.hex_dump);
  }

protected:
  scanner_udp_datagram_hexdumps::WithoutIntensities without_intensities_;
  MaxSizeRawData without_intensities_raw_;

  scanner_udp_datagram_hexdumps::WithDiagnostics with_diagnostics_;
  MaxSizeRawData with_diagnostics_raw_;
};

TEST_F(MonitoringFrameDeserializationTest, shouldDeserializeMonitoringFrameWithoutIntensitiesCorrectly)
{
  monitoring_frame::Message msg;
  ASSERT_NO_THROW(msg = monitoring_frame::deserialize(without_intensities_raw_, without_intensities_raw_.size()););
  EXPECT_EQ(msg, without_intensities_.expected_msg_);
}

TEST_F(MonitoringFrameDeserializationTest, shouldDeserializeMonitoringFrameWithDiagnosticsCorrectly)
{
  monitoring_frame::Message msg;
  ASSERT_NO_THROW(msg = monitoring_frame::deserialize(with_diagnostics_raw_, with_diagnostics_raw_.size()));
  EXPECT_TRUE(msg == with_diagnostics_.expected_msg_);
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnWrongOpCode)
{
  without_intensities_raw_.at(4) += 1;
  EXPECT_NO_THROW(monitoring_frame::deserialize(without_intensities_raw_, without_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnInvalidWorkingMode)
{
  without_intensities_raw_.at(8) = 0x03;
  EXPECT_NO_THROW(monitoring_frame::deserialize(without_intensities_raw_, without_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnInvalidTransactionType)
{
  without_intensities_raw_.at(12) = 0x06;
  EXPECT_NO_THROW(monitoring_frame::deserialize(without_intensities_raw_, without_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldPrintDebugMessageOnInvalidScannerId)
{
  without_intensities_raw_.at(16) = 0x04;
  EXPECT_NO_THROW(monitoring_frame::deserialize(without_intensities_raw_, without_intensities_raw_.size()););
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnUnknownFieldId)
{
  scanner_udp_datagram_hexdumps::WithUnknownFieldId with_unknown_field_id;
  const auto raw_frame_data = convertToMaxSizeRawData(with_unknown_field_id.hex_dump);
  const auto num_bytes = 2 * with_unknown_field_id.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);, monitoring_frame::FormatError);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeFieldLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeFieldLength with_too_large_field_length;
  const auto raw_frame_data = convertToMaxSizeRawData(with_too_large_field_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_field_length.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);, monitoring_frame::FormatError);
}

TEST_F(MonitoringFrameDeserializationTest, shouldThrowMonitoringFrameFormatErrorOnTooLargeScanCounterLength)
{
  scanner_udp_datagram_hexdumps::WithTooLargeScanCounterLength with_too_large_scan_counter_length;
  const auto raw_frame_data = convertToMaxSizeRawData(with_too_large_scan_counter_length.hex_dump);
  const auto num_bytes = 2 * with_too_large_scan_counter_length.hex_dump.size();

  monitoring_frame::Message msg;
  EXPECT_THROW(msg = monitoring_frame::deserialize(raw_frame_data, num_bytes);
               , monitoring_frame::FormatErrorScanCounterUnexpectedSize);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
