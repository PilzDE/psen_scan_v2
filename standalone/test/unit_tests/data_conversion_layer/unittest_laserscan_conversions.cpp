// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/data_conversion_layer/laserscan_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static data_conversion_layer::monitoring_frame::Message
createMsg(const util::TenthOfDegree from_theta = util::TenthOfDegree{ 10 },
          const util::TenthOfDegree resolution = util::TenthOfDegree{ 90 },
          const uint32_t scan_counter = uint32_t{ 42 })
{
  const std::vector<double> measurements{ 1., 2., 3., 4.5, 5., 42. };
  const std::vector<double> intensities{ 0., 4., 3., 1007., 508., 14000. };
  const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message> diagnostic_messages{};

  return data_conversion_layer::monitoring_frame::Message(
      from_theta, resolution, scan_counter, measurements, intensities, diagnostic_messages);
}

static std::vector<data_conversion_layer::monitoring_frame::Message> createMsgs(const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs;
  util::TenthOfDegree next_theta{ 10 };
  for (std::size_t i = 0; i < num_elements; ++i)
  {
    msgs.push_back(createMsg(next_theta));
    next_theta = next_theta + msgs[0].resolution() * static_cast<int>(msgs[0].measurements().size());
  }

  return msgs;
}

static data_conversion_layer::monitoring_frame::Message
copyMsgWithNewMeasurements(const data_conversion_layer::monitoring_frame::Message& msg,
                           const std::vector<double>& new_measurements)
{
  return data_conversion_layer::monitoring_frame::Message(msg.fromTheta(),
                                                          msg.resolution(),
                                                          msg.scanCounter(),
                                                          new_measurements,
                                                          msg.intensities(),
                                                          msg.diagnosticMessages());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanResolutionAfterConversion)
{
  const data_conversion_layer::monitoring_frame::Message frame{ createMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ frame }) }););

  EXPECT_EQ(frame.resolution(), scan_ptr->getScanResolution()) << "Resolution incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMinMaxScanAngleAfterConversion)
{
  const data_conversion_layer::monitoring_frame::Message frame{ createMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ frame }) }););

  const util::TenthOfDegree expected_max_scan_angle{ frame.fromTheta() +
                                                     frame.resolution() * (int)frame.measurements().size() };

  EXPECT_EQ(frame.fromTheta(), scan_ptr->getMinScanAngle()) << "Min scan-angle incorrect in LaserScan";
  EXPECT_EQ(expected_max_scan_angle, scan_ptr->getMaxScanAngle()) << "Max scan-angle incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMeasurementsAfterConversion)
{
  const data_conversion_layer::monitoring_frame::Message frame{ createMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ frame }) }););

  EXPECT_EQ(frame.measurements().size(), scan_ptr->getMeasurements().size());
  const auto mismatch_pair = std::mismatch(
      scan_ptr->getMeasurements().begin(), scan_ptr->getMeasurements().end(), frame.measurements().begin());
  EXPECT_EQ(scan_ptr->getMeasurements().end(), mismatch_pair.first)
      << "Measurement #" << std::distance(scan_ptr->getMeasurements().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectIntensitiesAfterConversion)
{
  const data_conversion_layer::monitoring_frame::Message frame{ createMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ frame }) }););

  EXPECT_EQ(frame.intensities().size(), scan_ptr->getIntensities().size());
  const auto mismatch_pair =
      std::mismatch(scan_ptr->getIntensities().begin(), scan_ptr->getIntensities().end(), frame.intensities().begin());
  EXPECT_EQ(scan_ptr->getIntensities().end(), mismatch_pair.first)
      << "Intensity #" << std::distance(scan_ptr->getIntensities().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingResolutions)
{
  auto msgs = createMsgs(6);
  msgs[1] = createMsg(msgs[1].fromTheta(), msgs[1].resolution() + util::TenthOfDegree(10));
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMissingFrames)
{
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan({}),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingThetaAngles)
{
  auto msgs = createMsgs(6);
  msgs[1] = createMsg(msgs[1].fromTheta() + util::TenthOfDegree(10));
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingScanCounters)
{
  auto msgs = createMsgs(6);
  msgs[1] = createMsg(msgs[1].fromTheta(), msgs[1].resolution(), msgs[1].scanCounter() + 1);
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, laserScanShouldContainAllScanInformationWhenBuildWithMultipleFrames)
{
  auto msgs = createMsgs(6);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(msgs) }););
  ASSERT_EQ(scan_ptr->getMeasurements().size(), msgs.size() * msgs[0].measurements().size());
  ASSERT_EQ(scan_ptr->getIntensities().size(), msgs.size() * msgs[0].intensities().size());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMeasurementsOrderedByThetaAngle)
{
  auto msgs = createMsgs(3);

  // Change first measurement value.
  auto new_measurements1 = msgs[0].measurements();
  new_measurements1[0] += 10;
  auto new_first_msg = copyMsgWithNewMeasurements(msgs[0], new_measurements1);

  // Change last measurement value.
  auto new_measurements2 = msgs[2].measurements();
  new_measurements2[new_measurements2.size() - 1] += 10;
  auto new_last_msg = copyMsgWithNewMeasurements(msgs[2], new_measurements2);

  // Build LaserScan with wrong message order.
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{
      data_conversion_layer::LaserScanConverter::toLaserScan({ msgs[1], new_last_msg, new_first_msg }) }););

  // Assert order was corrected.
  ASSERT_EQ(scan_ptr->getMeasurements().front(), new_measurements1.front());
  ASSERT_EQ(scan_ptr->getMeasurements().back(), new_measurements2.back());
}

TEST(LaserScanConversionTest, conversionShouldIgnoreEmptyFramesForMonitoringFramesValidation)
{
  using Message = data_conversion_layer::monitoring_frame::Message;
  using Tenth = util::TenthOfDegree;

  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<Message> messages = { Message(Tenth(2500), Tenth(2), 42, {}, {}, {}),
                                    Message(Tenth(0), Tenth(2), 42, {}, {}, {}),
                                    Message(Tenth(500), Tenth(2), 42, {}, {}, {}),
                                    Message(Tenth(1318), Tenth(2), 42, { 1., 2., 3. }, { 4., 5., 6. }, {}),
                                    Message(Tenth(1500), Tenth(2), 42, {}, {}, {}),
                                    Message(Tenth(2000), Tenth(2), 42, {}, {}, {}) };

  ASSERT_NO_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(messages));
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
