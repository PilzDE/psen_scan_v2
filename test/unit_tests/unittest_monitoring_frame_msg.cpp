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

#include <array>

#include <gtest/gtest.h>

#include "psen_scan_v2/istring_stream_builder.h"
#include "psen_scan_v2/monitoring_frame_format_error.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/string_stream_failure.h"
#include "psen_scan_v2/udp_frame_dumps.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
class MeasuresFieldTest : public ::testing::Test
{
protected:
  const std::array<uint16_t, 3> expected_measures_{ 44, 43, 42 };

  inline std::istringstream buildExpectedMeasuresStream(uint16_t length)
  {
    IStringStreamBuilder builder;
    builder.add(length);
    builder.add(expected_measures_.at(0));
    builder.add(expected_measures_.at(1));
    builder.add(expected_measures_.at(2));
    std::istringstream is{ builder.get() };
    return is;
  }
};

class FromRawTest : public ::testing::Test
{
protected:
  template <typename T>
  inline MaxSizeRawData buildRawData(const T hex_dump)
  {
    MaxSizeRawData ret;
    for (size_t i = 0; i < hex_dump.size(); i++)
    {
      ret.at(i) = static_cast<char>(hex_dump.at(i));
    }
    return ret;
  }
};

TEST(ScanCounterFieldTest, testReadSuccess)
{
  const uint16_t length = 4;
  const uint32_t expected_scan_counter = 2;

  IStringStreamBuilder builder;
  builder.add(length).add(expected_scan_counter);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_NO_THROW(ScanCounterField::readLengthAndPayload(is, scan_counter););
  EXPECT_EQ(expected_scan_counter, scan_counter);
}

TEST(ScanCounterFieldTest, testReadInvalidLengthFailure)
{
  const uint16_t length = 3;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_THROW(ScanCounterField::readLengthAndPayload(is, scan_counter);, MonitoringFrameFormatError);
}

TEST(ScanCounterFieldTest, testReadMissingPayloadFailure)
{
  const uint16_t length = 4;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_THROW(ScanCounterField::readLengthAndPayload(is, scan_counter);, StringStreamFailure);
}

TEST_F(MeasuresFieldTest, testReadSuccess)
{
  const uint16_t length = 6;

  std::istringstream is = buildExpectedMeasuresStream(length);

  std::vector<uint16_t> measures;

  EXPECT_NO_THROW(MeasuresField::readLengthAndPayload(is, measures););
  EXPECT_EQ(measures.at(0), 44);
  EXPECT_EQ(measures.at(1), 43);
  EXPECT_EQ(measures.at(2), 42);
}

TEST_F(MeasuresFieldTest, testReadMissingPayloadFailure)
{
  const uint16_t length = 6;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  std::vector<uint16_t> measures;
  EXPECT_THROW(MeasuresField::readLengthAndPayload(is, measures);, StringStreamFailure);
}

TEST_F(MeasuresFieldTest, testTooMuchMeasures)
{
  const uint16_t length = 4;
  std::istringstream is = buildExpectedMeasuresStream(length);
  std::vector<uint16_t> measures;
  EXPECT_NO_THROW(MeasuresField::readLengthAndPayload(is, measures););
}

TEST_F(MeasuresFieldTest, testTooFewMeasures)
{
  const uint16_t length = 8;
  std::istringstream is = buildExpectedMeasuresStream(length);
  std::vector<uint16_t> measures;
  EXPECT_THROW(MeasuresField::readLengthAndPayload(is, measures);, StringStreamFailure);
}

TEST(EndOfFrameFieldTest, testReadSuccess)
{
  const uint16_t length = 0;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  bool end_of_frame = false;
  EXPECT_NO_THROW(EndOfFrameField::setEndOfFrameMemberToTrue(is, end_of_frame););
  EXPECT_TRUE(end_of_frame);
}

TEST(EndOfFrameFieldTest, testReadIgnoreInvalidLength)
{
  const uint16_t length = 1;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  bool end_of_frame = false;
  EXPECT_NO_THROW(EndOfFrameField::setEndOfFrameMemberToTrue(is, end_of_frame););
  EXPECT_TRUE(end_of_frame);
}

TEST_F(FromRawTest, testReadSuccess)
{
  MaxSizeRawData data = buildRawData(monitoring_frame_without_intensities_hex_dump);
  MonitoringFrameMsg msg = MonitoringFrameMsg::fromRawData(data);

  // You get the following expected values from
  // the protocol description and the udp hex dump
  // You may need a hex calulator
  EXPECT_EQ(msg.fromTheta(), 1000);
  EXPECT_EQ(msg.resolution(), 10);

  size_t expected_measures_size = (0x65 - 1) / 2;
  EXPECT_EQ(msg.measures().size(), expected_measures_size);

  EXPECT_EQ(msg.measures().at(0), 681);
  EXPECT_EQ(msg.measures().at(1), 774);
  EXPECT_EQ(msg.measures().at(2), 636);
  EXPECT_EQ(msg.measures().at(3), 506);
  EXPECT_EQ(msg.measures().at(4), 496);
  EXPECT_EQ(msg.measures().at(30), 4063);
  EXPECT_EQ(msg.measures().at(45), 4074);
  EXPECT_EQ(msg.measures().at(49), 4657);
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
