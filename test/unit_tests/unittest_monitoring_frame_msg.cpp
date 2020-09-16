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

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(ScanCounterFieldTest, testReadSuccess)
{
  const uint16_t length = 4;
  const uint32_t expected_scan_counter = 2;

  IStringStreamBuilder builder;
  builder.add(length).add(expected_scan_counter);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_NO_THROW(ScanCounterField::read(is, scan_counter););
  EXPECT_EQ(expected_scan_counter, scan_counter);
}

TEST(ScanCounterFieldTest, testReadInvalidLengthFailure)
{
  const uint16_t length = 3;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_THROW(ScanCounterField::read(is, scan_counter);, MonitoringFrameFormatError);
}

TEST(ScanCounterFieldTest, testReadMissingPayloadFailure)
{
  const uint16_t length = 4;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  uint32_t scan_counter;
  EXPECT_THROW(ScanCounterField::read(is, scan_counter);, StringStreamFailure);
}

TEST(EndOfFrameFieldTest, testReadSuccess)
{
  const uint16_t length = 0;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  bool end_of_frame = false;
  EXPECT_NO_THROW(EndOfFrameField::read(is, end_of_frame););
  EXPECT_TRUE(end_of_frame);
}

TEST(EndOfFrameFieldTest, testReadIgnoreInvalidLength)
{
  const uint16_t length = 1;

  IStringStreamBuilder builder;
  builder.add(length);
  std::istringstream is{ builder.get() };

  bool end_of_frame = false;
  EXPECT_NO_THROW(EndOfFrameField::read(is, end_of_frame););
  EXPECT_TRUE(end_of_frame);
}
}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
