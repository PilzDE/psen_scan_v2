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
#include "psen_scan_v2/udp_frame_dumps.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(LaserScanConversionsTest, testToLaserScan)
{
  UDPFrameTestDataWithoutIntensities test_data;
  MaxSizeRawData raw_data{ convertToMaxSizeRawData(test_data.hex_dump) };
  MonitoringFrameMsg frame;
  ASSERT_NO_THROW(frame = MonitoringFrameMsg::fromRawData(raw_data););

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ toLaserScan(frame) }););

  EXPECT_DOUBLE_EQ(tenthDegreeToRad(frame.fromTheta()), scan_ptr->getMinScanAngle());
  EXPECT_DOUBLE_EQ(tenthDegreeToRad(frame.resolution()), scan_ptr->getScanResolution());

  const double max_scan_angle{ tenthDegreeToRad(frame.fromTheta() +
                                                frame.resolution() * (frame.measures().size() - 1)) };
  EXPECT_DOUBLE_EQ(max_scan_angle, scan_ptr->getMaxScanAngle());

  EXPECT_EQ(frame.measures().size(), scan_ptr->getMeasurements().size());
  const auto mismatch_pair =
      std::mismatch(scan_ptr->getMeasurements().begin(), scan_ptr->getMeasurements().end(), frame.measures().begin());
  EXPECT_EQ(scan_ptr->getMeasurements().end(), mismatch_pair.first)
      << "Measure number " << std::distance(scan_ptr->getMeasurements().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}
}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
