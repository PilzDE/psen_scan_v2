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
#include "psen_scan_v2_standalone/util/monitoring_frame_msg_builder.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static const int64_t DEFAULT_TIMESTAMP{ 4500000 };
static const int64_t EXPECTED_TIMESTAMP_AFTER_CONVERSION{ 750000 };  // 4500000 - ((45/360) * 30*10^6)

static data_conversion_layer::monitoring_frame::MessageStamped
createStampedMsg(const int64_t timestamp = DEFAULT_TIMESTAMP,
                 const util::TenthOfDegree from_theta = util::TenthOfDegree{ 10 },
                 const util::TenthOfDegree resolution = util::TenthOfDegree{ 90 },
                 const uint32_t scan_counter = uint32_t{ 42 },
                 const uint8_t active_zoneset = uint8_t{ 0 })
{
  MonitoringFrameMsgBuilder msg_builder;
  msg_builder.fromTheta(from_theta)
      .resolution(resolution)
      .scanCounter(scan_counter)
      .activeZoneset(active_zoneset)
      .measurements({ 1., 2., 3., 4.5, 5., 42. })
      .intensities({ 0., 4., 3., 1007., 508., 14000. });

  return data_conversion_layer::monitoring_frame::MessageStamped(msg_builder.build(), timestamp);
}

static std::vector<data_conversion_layer::monitoring_frame::MessageStamped>
createStampedMsgs(const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::MessageStamped> msgs;
  util::TenthOfDegree next_theta{ 10 };
  int64_t next_timestamp{ DEFAULT_TIMESTAMP };
  for (std::size_t i = 0; i < num_elements; ++i)
  {
    msgs.push_back(createStampedMsg(next_timestamp, next_theta));
    next_timestamp += DEFAULT_TIMESTAMP;
    next_theta = next_theta + msgs[0].msg_.resolution_ * static_cast<int>(msgs[0].msg_.measurements_.size());
  }

  return msgs;
}

static void addStampedMsgWithActiveZone(std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& msgs,
                                        uint8_t active_zone)
{
  msgs.push_back(
      createStampedMsg(DEFAULT_TIMESTAMP * msgs.size(),
                       msgs[0].msg_.from_theta_ +
                           msgs[0].msg_.resolution_ * static_cast<int>(msgs[0].msg_.measurements_.size()) * msgs.size(),
                       msgs[0].msg_.resolution_,
                       msgs[0].msg_.scan_counter_.value(),
                       active_zone));
}

static data_conversion_layer::monitoring_frame::MessageStamped
copyStampedMsgWithNewTimestamp(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg,
                               const int64_t new_timestamp)
{
  return data_conversion_layer::monitoring_frame::MessageStamped(stamped_msg.msg_, new_timestamp);
}

static data_conversion_layer::monitoring_frame::MessageStamped
copyStampedMsgWithNewMeasurements(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg,
                                  const std::vector<double>& new_measurements)
{
  data_conversion_layer::monitoring_frame::MessageStamped copied_msg{ stamped_msg };
  copied_msg.msg_.measurements_ = new_measurements;
  return copied_msg;
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanResolutionAfterConversion)
{
  const auto stamped_msg{ createStampedMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(stamped_msg.msg_.resolution_, scan_ptr->getScanResolution()) << "Resolution incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMinMaxScanAngleAfterConversion)
{
  const auto stamped_msg{ createStampedMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  const util::TenthOfDegree expected_max_scan_angle{
    stamped_msg.msg_.from_theta_ + stamped_msg.msg_.resolution_ * ((int)stamped_msg.msg_.measurements_.size() - 1)
  };

  EXPECT_EQ(stamped_msg.msg_.from_theta_, scan_ptr->getMinScanAngle()) << "Min scan-angle incorrect in LaserScan";
  EXPECT_EQ(expected_max_scan_angle, scan_ptr->getMaxScanAngle()) << "Max scan-angle incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMeasurementsAfterConversion)
{
  const auto stamped_msg{ createStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(stamped_msg.msg_.measurements_.size(), scan_ptr->getMeasurements().size());
  const auto mismatch_pair = std::mismatch(
      scan_ptr->getMeasurements().begin(), scan_ptr->getMeasurements().end(), stamped_msg.msg_.measurements_.begin());
  EXPECT_EQ(scan_ptr->getMeasurements().end(), mismatch_pair.first)
      << "Measurement #" << std::distance(scan_ptr->getMeasurements().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectIntensitiesAfterConversion)
{
  const auto stamped_msg{ createStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(stamped_msg.msg_.intensities_.size(), scan_ptr->getIntensities().size());
  const auto mismatch_pair = std::mismatch(
      scan_ptr->getIntensities().begin(), scan_ptr->getIntensities().end(), stamped_msg.msg_.intensities_.begin());
  EXPECT_EQ(scan_ptr->getIntensities().end(), mismatch_pair.first)
      << "Intensity #" << std::distance(scan_ptr->getIntensities().begin(), mismatch_pair.first)
      << " in LaserScan is: " << *(mismatch_pair.first) << ", but expected: " << *(mismatch_pair.second);
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanCounterAfterConversion)
{
  const auto stamped_msg{ createStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  ASSERT_TRUE(stamped_msg.msg_.scan_counter_.is_initialized());
  EXPECT_EQ(stamped_msg.msg_.scan_counter_.value(), scan_ptr->getScanCounter());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectTimestampAfterConversion)
{
  const auto stamped_msg{ createStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->getTimestamp());
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingResolutions)
{
  auto stamped_msgs = createStampedMsgs(6);
  stamped_msgs[1] = createStampedMsg(stamped_msgs[1].stamp_,
                                     stamped_msgs[1].msg_.from_theta_,
                                     stamped_msgs[1].msg_.resolution_ + util::TenthOfDegree(10));
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMissingFrames)
{
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan({}),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingThetaAngles)
{
  auto stamped_msgs = createStampedMsgs(6);
  stamped_msgs[1] =
      createStampedMsg(stamped_msgs[1].stamp_, stamped_msgs[1].msg_.from_theta_ + util::TenthOfDegree(10));
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingScanCounters)
{
  auto stamped_msgs = createStampedMsgs(6);
  stamped_msgs[1] = createStampedMsg(stamped_msgs[1].stamp_,
                                     stamped_msgs[1].msg_.from_theta_,
                                     stamped_msgs[1].msg_.resolution_,
                                     *stamped_msgs[1].msg_.scan_counter_ + 1);
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, laserScanShouldContainAllScanInformationWhenBuildWithMultipleFrames)
{
  auto stamped_msgs = createStampedMsgs(6);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  ASSERT_EQ(scan_ptr->getMeasurements().size(), stamped_msgs.size() * stamped_msgs[0].msg_.measurements_.size());
  ASSERT_EQ(scan_ptr->getIntensities().size(), stamped_msgs.size() * stamped_msgs[0].msg_.intensities_.size());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectTimestampWhenBuildingWithMulitpleFrames)
{
  auto stamped_msgs = createStampedMsgs(3);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->getTimestamp());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMeasurementsOrderedByThetaAngle)
{
  auto stamped_msgs = createStampedMsgs(3);

  // Change first measurement value.
  auto new_measurements1 = stamped_msgs[0].msg_.measurements_;
  new_measurements1[0] += 10;
  auto new_first_msg = copyStampedMsgWithNewMeasurements(stamped_msgs[0], new_measurements1);

  // Change last measurement value.
  auto new_measurements2 = stamped_msgs[2].msg_.measurements_;
  new_measurements2[new_measurements2.size() - 1] += 10;
  auto new_last_msg = copyStampedMsgWithNewMeasurements(stamped_msgs[2], new_measurements2);

  // Build LaserScan with wrong message order.
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(scan_ptr.reset(new LaserScan{
      data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msgs[1], new_last_msg, new_first_msg }) }););

  // Assert order was corrected.
  ASSERT_EQ(scan_ptr->getMeasurements().front(), new_measurements1.front());
  ASSERT_EQ(scan_ptr->getMeasurements().back(), new_measurements2.back());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMinimalTimestamp)
{
  auto stamped_msgs = createStampedMsgs(3);

  // swap timestamps
  const auto swapped_timestamp{ stamped_msgs[0].stamp_ };
  stamped_msgs[0] = copyStampedMsgWithNewTimestamp(stamped_msgs[0], stamped_msgs[1].stamp_);
  stamped_msgs[1] = copyStampedMsgWithNewTimestamp(stamped_msgs[1], swapped_timestamp);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->getTimestamp());
}

TEST(LaserScanConversionsTest, laserScanShouldContainActiveZoneset)
{
  auto stamped_msgs = createStampedMsgs(5);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(0, scan_ptr->getActiveZoneset());
}

TEST(LaserScanConversionsTest, laserScanShouldContainActiveZonesetOfLastMsg)
{
  auto stamped_msgs = createStampedMsgs(1);
  addStampedMsgWithActiveZone(stamped_msgs, 3);
  addStampedMsgWithActiveZone(stamped_msgs, 4);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(4, scan_ptr->getActiveZoneset());
}

using Message = data_conversion_layer::monitoring_frame::Message;
using Stamped = data_conversion_layer::monitoring_frame::MessageStamped;
using Tenth = util::TenthOfDegree;
using Builder = MonitoringFrameMsgBuilder;

TEST(LaserScanConversionTest, conversionShouldIgnoreEmptyFramesForMonitoringFramesValidation)
{
  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<Stamped> stamped_msgs = {
    Stamped(Builder().fromTheta(Tenth(2500)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 3),
    Stamped(Builder().fromTheta(Tenth(0)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 4),
    Stamped(Builder().fromTheta(Tenth(500)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 5),
    Stamped(Builder()
                .fromTheta(Tenth(1318))
                .resolution(Tenth(2))
                .scanCounter(42)
                .activeZoneset(1)
                .measurements({ 1., 2., 3. })
                .intensities({ 4., 5., 6. }),
            6),
    Stamped(Builder().fromTheta(Tenth(1500)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 7),
    Stamped(Builder().fromTheta(Tenth(2000)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 8)
  };

  ASSERT_NO_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs));
}

TEST(LaserScanConversionsTest, conversionShouldIgnoreEmptyFramesForTimestampsComputation)
{
  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<Stamped> stamped_msgs = {
    Stamped(Builder().fromTheta(Tenth(2500)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 3),
    Stamped(Builder().fromTheta(Tenth(0)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 4),
    Stamped(Builder().fromTheta(Tenth(500)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 5),
    Stamped(Builder()
                .fromTheta(Tenth(1318))
                .resolution(Tenth(2))
                .scanCounter(42)
                .activeZoneset(1)
                .measurements({ 1., 2., 3. })
                .intensities({ 4., 5., 6. }),
            40000),
    Stamped(Builder().fromTheta(Tenth(1500)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 7),
    Stamped(Builder().fromTheta(Tenth(2000)).resolution(Tenth(2)).scanCounter(42).activeZoneset(1), 8)
  };
  const int64_t expected_stamp{ 6667 };  // 40000 - ((0.4/360) * 30*10^6)

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  EXPECT_EQ(expected_stamp, scan_ptr->getTimestamp());
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
