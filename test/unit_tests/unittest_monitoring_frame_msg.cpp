// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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

#include <algorithm>
#include <array>
#include <memory>

#include <gtest/gtest.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/istring_stream_builder.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2_test;

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
  ASSERT_NO_THROW(header_ptr.reset(new FieldHeader{ MonitoringFrameMsg::readFieldHeader(is, max_num_bytes) }););
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

  EXPECT_THROW(MonitoringFrameMsg::readFieldHeader(is, max_num_bytes);, raw_processing::StringStreamFailure);
}

class MonitoringFrameMsgTest : public ::testing::Test
{
protected:
  inline std::istringstream buildExpectedMeasuresStream()
  {
    IStringStreamBuilder builder;
    for (const auto& measure : expected_measures_)
    {
      builder.add(static_cast<uint16_t>(measure * 1000.));
    }
    return builder.get();
  }

  inline bool expectMeasuresPartEqual(const std::vector<double>& measures)
  {
    return std::equal(measures.begin(), measures.end(), expected_measures_.begin());
  }

  inline bool expectMeasuresEqual(const std::vector<double>& measures)
  {
    return (measures.size() == expected_measures_.size() && expectMeasuresPartEqual(measures));
  }

protected:
  const std::array<double, 3> expected_measures_{ 4.4, 4.3, 4.2 };
};

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
  ASSERT_NO_THROW(msg = MonitoringFrameMsg::deserialize(raw_frame_data_, num_bytes_););

  EXPECT_EQ(msg, test_data_.msg_);
}

TEST_F(MonitoringFrameMsgDeserializeTest, testWrongOpCode)
{
  raw_frame_data_.at(4) += 1;
  EXPECT_NO_THROW(MonitoringFrameMsg::deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testInvalidWorkingMode)
{
  raw_frame_data_.at(8) = 0x03;
  EXPECT_NO_THROW(MonitoringFrameMsg::deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testInvalidTransactionType)
{
  raw_frame_data_.at(12) = 0x06;
  EXPECT_NO_THROW(MonitoringFrameMsg::deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testInvalidScannerId)
{
  raw_frame_data_.at(16) = 0x04;
  EXPECT_NO_THROW(MonitoringFrameMsg::deserialize(raw_frame_data_, num_bytes_););
}

TEST_F(MonitoringFrameMsgDeserializeTest, testUnknownFieldId)
{
  UDPFrameTestDataWithUnknownFieldId test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = MonitoringFrameMsg::deserialize(raw_frame_data, num_bytes);
               , MonitoringFrameMsg::MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameMsgDeserializeTest, testTooLargeFieldLength)
{
  UDPFrameTestDataWithTooLargeFieldLength test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = MonitoringFrameMsg::deserialize(raw_frame_data, num_bytes);
               , MonitoringFrameMsg::MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameMsgDeserializeTest, testTooLargeScanCounterLength)
{
  UDPFrameTestDataWithTooLargeScanCounterLength test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = MonitoringFrameMsg::deserialize(raw_frame_data, num_bytes);
               , MonitoringFrameMsg::MonitoringFrameFormatErrorScanCounterUnexpectedSize);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualSucces)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 1, 2, 3 });
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(10), 42, { 1, 2, 3 });
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualEmptySuccess)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, {});
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(10), 42, {});
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareMeasuresNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 42, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareFromThetaNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(101), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareResolutionNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(11), 42, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareScanCounterNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(10), 43, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareNotEqualEmpty)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, {});
  MonitoringFrameMsg msg1(TenthOfDegree(111), TenthOfDegree(10), 42, {});
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccess)
{
  MonitoringFrameMsg msg(TenthOfDegree(1234), TenthOfDegree(56), 78, { 45, 44, 43, 42 });
  std::stringstream output;
  output << msg;
  EXPECT_EQ(output.str(),
            "MonitoringFrameMsg(fromTheta = TenthOfDegree(1234), resolution = TenthOfDegree(56), scanCounter = 78, "
            "measures = { 45, 44, 43, 42 })");
}

}  // namespace psen_scan_v2

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
