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
#include "psen_scan_v2/monitoring_frame_format_error.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/string_stream_failure.h"
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

  IStringStreamBuilder builder;
  builder.add(id);
  builder.add(length);
  std::istringstream is{ builder.get() };

  std::unique_ptr<FieldHeader> header_ptr;
  ASSERT_NO_THROW(header_ptr.reset(new FieldHeader{ MonitoringFrameMsg::readFieldHeader(is) }););
  EXPECT_EQ(id, header_ptr->id());
  EXPECT_EQ(expected_length, header_ptr->length());
}

TEST(FieldHeaderTest, testReadHeaderTooShortFailure)
{
  uint16_t too_short_header;

  IStringStreamBuilder builder;
  builder.add(too_short_header);
  std::istringstream is{ builder.get() };

  EXPECT_THROW(MonitoringFrameMsg::readFieldHeader(is);, StringStreamFailure);
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

TEST_F(MonitoringFrameMsgTest, testReadScanCounterSuccess)
{
  const uint16_t length = 4;
  const uint32_t expected_scan_counter = 2;

  IStringStreamBuilder builder;
  builder.add(expected_scan_counter);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  ASSERT_NO_THROW(MonitoringFrameMsg::readScanCounter(is, scan_counter, length));
  EXPECT_EQ(expected_scan_counter, scan_counter);
}

TEST_F(MonitoringFrameMsgTest, testReadScanCounterInvalidLengthFailure)
{
  const uint16_t length = 3;
  const uint32_t expected_scan_counter = 2;

  IStringStreamBuilder builder;
  builder.add(expected_scan_counter);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_THROW(MonitoringFrameMsg::readScanCounter(is, scan_counter, length);, MonitoringFrameFormatError);
}

TEST_F(MonitoringFrameMsgTest, testReadScanCounterMissingPayloadFailure)
{
  const uint16_t length = 4;
  std::istringstream is;
  uint32_t scan_counter;
  EXPECT_THROW(MonitoringFrameMsg::readScanCounter(is, scan_counter, length);, StringStreamFailure);
}

TEST_F(MonitoringFrameMsgTest, testReadMeasuresSuccess)
{
  const uint16_t length = 2 * expected_measures_.size();
  std::istringstream is = buildExpectedMeasuresStream();

  std::vector<double> measures;
  ASSERT_NO_THROW(MonitoringFrameMsg::readMeasures(is, measures, length););
  EXPECT_TRUE(expectMeasuresEqual(measures));
}

TEST_F(MonitoringFrameMsgTest, testReadMeasuresMissingPayloadFailure)
{
  const uint16_t length = 2 * expected_measures_.size();
  std::istringstream is;
  std::vector<double> measures;
  EXPECT_THROW(MonitoringFrameMsg::readMeasures(is, measures, length);, StringStreamFailure);
}

TEST_F(MonitoringFrameMsgTest, testReadMeasuresTooMuchMeasures)
{
  const uint16_t length = 2 * expected_measures_.size() - 1;
  std::istringstream is = buildExpectedMeasuresStream();

  std::vector<double> measures;
  ASSERT_NO_THROW(MonitoringFrameMsg::readMeasures(is, measures, length););
  EXPECT_TRUE(expectMeasuresPartEqual(measures));
}

TEST_F(MonitoringFrameMsgTest, testReadMeasuresTooFewMeasures)
{
  const uint16_t length = 2 * (expected_measures_.size() + 1);
  std::istringstream is = buildExpectedMeasuresStream();
  std::vector<double> measures;
  EXPECT_THROW(MonitoringFrameMsg::readMeasures(is, measures, length);, StringStreamFailure);
}

TEST_F(MonitoringFrameMsgTest, testFromRawDataSuccess)
{
  UDPFrameTestDataWithoutIntensities test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);

  MonitoringFrameMsg msg;
  ASSERT_NO_THROW(msg = MonitoringFrameMsg::fromRawData(raw_frame_data););

  EXPECT_DOUBLE_EQ(msg.fromTheta(), tenthDegreeToRad(test_data.from_theta));
  EXPECT_DOUBLE_EQ(msg.resolution(), tenthDegreeToRad(test_data.resolution));
  EXPECT_EQ(msg.scanCounter(), test_data.scan_counter);

  const auto measures = msg.measures();
  EXPECT_EQ(measures.size(), test_data.number_of_measures);
  for (const auto& index_value_pair : test_data.measures)
  {
    EXPECT_DOUBLE_EQ(measures.at(index_value_pair.first), index_value_pair.second / 1000.);
  }
}

TEST_F(MonitoringFrameMsgTest, testFromRawDataFailureUnknownFieldId)
{
  UDPFrameTestDataWithUnknownFieldId test_data;
  const auto raw_frame_data = convertToMaxSizeRawData(test_data.hex_dump);

  MonitoringFrameMsg msg;
  EXPECT_THROW(msg = MonitoringFrameMsg::fromRawData(raw_frame_data);, MonitoringFrameFormatError);
}
}  // namespace psen_scan_v2

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
