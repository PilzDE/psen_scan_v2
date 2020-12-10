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

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/angle_conversions.h"
#include "psen_scan_v2_standalone/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/raw_processing.h"

#include "psen_scan_v2/istring_stream_builder.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2_test;
using namespace psen_scan_v2_standalone;

class MonitoringFrameMsgTest : public ::testing::Test
{
protected:
  inline std::istringstream buildExpectedMeasurementsStream()
  {
    IStringStreamBuilder builder;
    for (const auto& measurement : expected_measurements_)
    {
      builder.add(static_cast<uint16_t>(measurement * 1000.));
    }
    return builder.get();
  }

  inline bool expectMeasurementsPartEqual(const std::vector<double>& measurements)
  {
    return std::equal(measurements.begin(), measurements.end(), expected_measurements_.begin());
  }

  inline bool expectMeasurementsEqual(const std::vector<double>& measurements)
  {
    return (measurements.size() == expected_measurements_.size() && expectMeasurementsPartEqual(measurements));
  }

protected:
  const std::array<double, 3> expected_measurements_{ 4.4, 4.3, 4.2 };
};

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualSucces)
{
  const monitoring_frame::Message msg0(TenthOfDegree(100), TenthOfDegree(10), 42, { 1, 2, 3 });
  const monitoring_frame::Message msg1(TenthOfDegree(100), TenthOfDegree(10), 42, { 1, 2, 3 });
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualIntensitiesSucces)
{
  const monitoring_frame::Message msg0(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  const monitoring_frame::Message msg1(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualEmptySuccess)
{
  const monitoring_frame::Message msg0(TenthOfDegree(100), TenthOfDegree(10), 42, {});
  const monitoring_frame::Message msg1(TenthOfDegree(100), TenthOfDegree(10), 42, {});
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualIntensitiesEmptySucces)
{
  const monitoring_frame::Message msg0(TenthOfDegree(100), TenthOfDegree(10), 42, {}, {}, {});
  const monitoring_frame::Message msg1(TenthOfDegree(100), TenthOfDegree(10), 42, {}, {}, {});
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareMeasurementsNotEqual)
{
  const monitoring_frame::Message msg0(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 42, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  const monitoring_frame::Message msg1(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareIntensitiesNotEqual)
{
  const monitoring_frame::Message msg0(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 42, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  const monitoring_frame::Message msg1(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareFromThetaNotEqual)
{
  const monitoring_frame::Message msg0(
      TenthOfDegree(42),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  const monitoring_frame::Message msg1(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareResolutionNotEqual)
{
  const monitoring_frame::Message msg0(
      TenthOfDegree(100),
      TenthOfDegree(42),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  const monitoring_frame::Message msg1(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareScanCounterNotEqual)
{
  const monitoring_frame::Message msg0(
      TenthOfDegree(100),
      TenthOfDegree(10),
      42,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  const monitoring_frame::Message msg1(
      TenthOfDegree(100),
      TenthOfDegree(10),
      1,
      { 1, 2, 3 },
      { 10, 20, 30 },
      { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(5, 3)) });
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgEqualityTest, testCompareNotEqualEmpty)
{
  const monitoring_frame::Message msg0(TenthOfDegree(100), TenthOfDegree(42), 42, {}, {}, {});
  const monitoring_frame::Message msg1(TenthOfDegree(100), TenthOfDegree(42), 0, {}, {}, {});
  EXPECT_FALSE(msg0 == msg1) << "Comparision between\n\t" << msg0 << "\nand\n\t" << msg1
                             << "\nexpected to be false but was true";
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccess)
{
  monitoring_frame::Message msg(TenthOfDegree(1234), TenthOfDegree(56), 78, { 45, 44, 43, 42 });

// For compatibility with different ubuntu versions (resp. fmt), we need to take account of changes in
// the default formatting of floating point numbers
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "measurements = {45.0, 44.0, 43.0, 42.0}, intensities = {}, diagnostics = {})");
#else
  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "measurements = {45, 44, 43, 42}, intensities = {}, diagnostics = {})");
#endif
}

}  // namespace psen_scan_v2

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
