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

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"
#include "psen_scan_v2/istring_stream_builder.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2_test;

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
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareFromThetaNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(101), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareResolutionNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(11), 42, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareScanCounterNotEqual)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 45, 44, 43, 42 });
  MonitoringFrameMsg msg1(TenthOfDegree(100), TenthOfDegree(10), 43, { 45, 44, 43, 42 });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareNotEqualEmpty)
{
  MonitoringFrameMsg msg0(TenthOfDegree(100), TenthOfDegree(10), 42, {});
  MonitoringFrameMsg msg1(TenthOfDegree(111), TenthOfDegree(10), 42, {});
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccess)
{
  MonitoringFrameMsg msg(TenthOfDegree(1234), TenthOfDegree(56), 78, { 45, 44, 43, 42 });
  EXPECT_EQ(fmt::format("{}", msg),
            "MonitoringFrameMsg(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "measures = {45.0, 44.0, 43.0, 42.0}, diagnostics = {})");
}

}  // namespace psen_scan_v2

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
