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

#include "psen_scan_v2/monitoring_frame_serialization.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"
#include "psen_scan_v2/istring_stream_builder.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(FieldHeaderTest, testGetIdAndLength)
{
  uint8_t id = 5;
  uint16_t length = 7;
  FieldHeader header(id, length);
  EXPECT_EQ(id, header.id());
  EXPECT_EQ(length, header.length());
}

TEST(FieldHeaderTest, testReadSuccess)
{
  uint8_t id = 5;
  uint16_t length = 7;
  uint16_t expected_length = length - 1;
  uint16_t max_num_bytes = 9;

  IStringStreamBuilder builder;
  builder.add(id);
  builder.add(length);
  std::istringstream is{ builder.get() };

  std::unique_ptr<FieldHeader> header_ptr;
  ASSERT_NO_THROW(header_ptr.reset(new FieldHeader{ readFieldHeader(is, max_num_bytes) }););
  EXPECT_EQ(id, header_ptr->id());
  EXPECT_EQ(expected_length, header_ptr->length());
}

TEST(FieldHeaderTest, testReadHeaderTooShortFailure)
{
  uint16_t too_short_header;
  uint16_t max_num_bytes = 2;

  IStringStreamBuilder builder;
  builder.add(too_short_header);
  std::istringstream is{ builder.get() };

  EXPECT_THROW(readFieldHeader(is, max_num_bytes);, raw_processing::StringStreamFailure);
}

TEST(MonitoringFrameSerializationTest, testUDPFrameTestDataWithoutIntensitiesSuccess)
{
  // Load testdata from dump
  UDPFrameTestDataWithoutIntensities test_data;
  DynamicSizeRawData serialized_monitoring_frame_message = serialize(test_data.msg_);

  EXPECT_EQ(test_data.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < test_data.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), test_data.hex_dump.at(i)) << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, testUDPFrameTestDataWithoutMeasurementsAndIntensitiesSuccess)
{
  // Load testdata from dump
  UDPFrameTestDataWithoutMeasurementsAndIntensities test_data;
  DynamicSizeRawData serialized_monitoring_frame_message = serialize(test_data.msg_);

  EXPECT_EQ(test_data.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < test_data.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), test_data.hex_dump.at(i)) << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, testSerializationInvariance)
{
  UDPFrameTestDataWithoutIntensities test_data;
  DynamicSizeRawData raw = serialize(test_data.msg_);

  MonitoringFrameMsg deserialized_msg = deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(test_data.msg_, deserialized_msg);
}

TEST(MonitoringFrameSerializationTest, testSerializationInvariance2)
{
  MonitoringFrameMsg msg(TenthOfDegree(25), TenthOfDegree(1), 456, { 10, 20, 30, 40 });

  DynamicSizeRawData raw = serialize(msg);

  MonitoringFrameMsg deserialized_msg = deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(msg, deserialized_msg);
}

class MonitoringFrameMsgDeserializeTest : public ::testing::Test
{
protected:
  MonitoringFrameMsgDeserializeTest()
  {
    raw_frame_data_ = convertToMaxSizeRawData(test_data_.hex_dump);
  }

protected:
  UDPFrameTestDataWithoutIntensities test_data_;
  MaxSizeRawData raw_frame_data_;
  std::size_t num_bytes_{ 2 * test_data_.hex_dump.size() };
};

TEST_F(MonitoringFrameMsgDeserializeTest, testDeserializationSuccess)
{
  MonitoringFrameMsg msg;
  ASSERT_NO_THROW(msg = deserialize(raw_frame_data_, num_bytes_););

  EXPECT_EQ(msg, test_data_.msg_);
}

TEST_F(MonitoringFrameMsgDeserializeTest, testWrongOpCode)
{
  raw_frame_data_.at(4) += 1;
  EXPECT_NO_THROW(deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testInvalidWorkingMode)
{
  raw_frame_data_.at(8) = 0x03;
  EXPECT_NO_THROW(deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testInvalidTransactionType)
{
  raw_frame_data_.at(12) = 0x06;
  EXPECT_NO_THROW(deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testInvalidScannerId)
{
  raw_frame_data_.at(16) = 0x04;
  EXPECT_NO_THROW(deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testUnknownFieldId)
{
  UDPFrameTestDataWithUnknownFieldId test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = deserialize(raw_frame_data, num_bytes);, MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameMsgDeserializeTest, testTooLargeFieldLength)
{
  UDPFrameTestDataWithTooLargeFieldLength test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = deserialize(raw_frame_data, num_bytes);, MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameMsgDeserializeTest, testTooLargeScanCounterLength)
{
  UDPFrameTestDataWithTooLargeScanCounterLength test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = deserialize(raw_frame_data, num_bytes);, MonitoringFrameFormatErrorScanCounterUnexpectedSize);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}