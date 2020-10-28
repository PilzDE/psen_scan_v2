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

#include <algorithm>
#include <cstdint>
#include <memory>

#include <gtest/gtest.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/laserscan_conversions.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_scanner_data.h"

#include "psen_scan_v2/raw_data_array_conversion.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
static MonitoringFrameMsg createMsg()
{
  const TenthOfDegree from_theta{ 10 };
  const TenthOfDegree resolution{ 90 };
  const uint32_t scan_counter{ 42 };
  const std::vector<double> measurements{ 1., 2., 3., 4.5, 5., 42. };
  const std::vector<double> intensities{ 0., 4., 3., 1007., 508., 14000. };
  const std::vector<MonitoringFrameDiagnosticMessage> diagnostic_messages{};

  return MonitoringFrameMsg(from_theta, resolution, scan_counter, measurements, intensities, diagnostic_messages);
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanResolutionAfterConversion)
{
  const MonitoringFrameMsg frame{ createMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ toLaserScan(frame) }););

  EXPECT_EQ(frame.resolution(), scan_ptr->getScanResolution()) << "Resolution incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMinMaxScanAngleAfterConversion)
{
  const MonitoringFrameMsg frame{ createMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ toLaserScan(frame) }););

  const TenthOfDegree expected_max_scan_angle{ frame.fromTheta() + frame.resolution() * frame.measures().size() };

  EXPECT_EQ(frame.fromTheta(), scan_ptr->getMinScanAngle()) << "Min scan-angle incorrect in LaserScan";
  EXPECT_EQ(expected_max_scan_angle, scan_ptr->getMaxScanAngle()) << "Max scan-angle incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMeasurementsAfterConversion)
{
  const MonitoringFrameMsg frame{ createMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ toLaserScan(frame) }););

  EXPECT_EQ(frame.measures().size(), scan_ptr->getMeasurements().size());
  const auto mismatch_pair =
      std::mismatch(scan_ptr->getMeasurements().begin(), scan_ptr->getMeasurements().end(), frame.measures().begin());
  EXPECT_EQ(scan_ptr->getMeasurements().end(), mismatch_pair.first)
      << "Measure #" << std::distance(scan_ptr->getMeasurements().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectIntensitiesAfterConversion)
{
  const MonitoringFrameMsg frame{ createMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ toLaserScan(frame) }););

  EXPECT_EQ(frame.intensities().size(), scan_ptr->getIntensities().size());
  const auto mismatch_pair =
      std::mismatch(scan_ptr->getIntensities().begin(), scan_ptr->getIntensities().end(), frame.intensities().begin());
  EXPECT_EQ(scan_ptr->getIntensities().end(), mismatch_pair.first)
      << "Intensity #" << std::distance(scan_ptr->getIntensities().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
