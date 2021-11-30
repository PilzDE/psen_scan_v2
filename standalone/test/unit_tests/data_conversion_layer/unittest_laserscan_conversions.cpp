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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/data_conversion_layer/laserscan_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

#define CHANGE_MSG_PROPERTY(msg, property, offset)                                                                     \
  msg = MessageBuilder()                                                                                               \
            .scannerId(msg.scannerId())                                                                                \
            .fromTheta(msg.fromTheta())                                                                                \
            .resolution(msg.resolution())                                                                              \
            .scanCounter(msg.scanCounter())                                                                            \
            .activeZoneset(msg.activeZoneset())                                                                        \
            .measurements(msg.measurements())                                                                          \
            .intensities(msg.intensities())                                                                            \
            .property(msg.property() + offset)

#define CHANGE_MSG_VECTOR_PROPERTY(msg, property, index, offset)                                                       \
  {                                                                                                                    \
    auto prop = msg.property();                                                                                        \
    prop.at(index) += offset;                                                                                          \
    msg = MessageBuilder()                                                                                             \
              .scannerId(msg.scannerId())                                                                              \
              .fromTheta(msg.fromTheta())                                                                              \
              .resolution(msg.resolution())                                                                            \
              .scanCounter(msg.scanCounter())                                                                          \
              .activeZoneset(msg.activeZoneset())                                                                      \
              .measurements(msg.measurements())                                                                        \
              .intensities(msg.intensities())                                                                          \
              .property(prop);                                                                                         \
  }                                                                                                                    \
  while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
using data_conversion_layer::monitoring_frame::MessageBuilder;
using data_conversion_layer::monitoring_frame::MessageStamped;

static const int64_t DEFAULT_TIMESTAMP{ 4500000 };
static const int64_t EXPECTED_TIMESTAMP_AFTER_CONVERSION{ 4400000 };  // 4500000 - ((1.2/360) * 30*10^6)

static MessageBuilder createDefaultMsgBuilder()
{
  return MessageBuilder()
      .fromTheta(util::TenthOfDegree{ 10 })
      .resolution(util::TenthOfDegree{ 2 })
      .scanCounter(42)
      .activeZoneset(0)
      .measurements({ 1., 2., 3., 4.5, 5., 42., .4 })
      .intensities({ 0., 4., 3., 1007., 508., 14000., .4 });
}

static MessageStamped createDefaultStampedMsg(const int64_t timestamp = DEFAULT_TIMESTAMP)
{
  return MessageStamped(createDefaultMsgBuilder(), timestamp);
}

static void addStampedMsg(std::vector<MessageStamped>& msgs)
{
  if (msgs.empty())
  {
    msgs.push_back(createDefaultStampedMsg());
  }
  else
  {
    auto builder{ createDefaultMsgBuilder() };
    const auto& last_stamped_msg{ msgs.back() };
    builder.fromTheta(last_stamped_msg.msg_.fromTheta() +
                      last_stamped_msg.msg_.resolution() *
                          static_cast<int>(last_stamped_msg.msg_.measurements().size()));
    msgs.push_back(MessageStamped(builder, last_stamped_msg.stamp_ + DEFAULT_TIMESTAMP));
  }
}

static std::vector<MessageStamped> createValidStampedMsgs(const std::size_t num_elements)
{
  std::vector<MessageStamped> msgs;
  for (std::size_t i = 0; i < num_elements; ++i)
  {
    addStampedMsg(msgs);
  }
  return msgs;
}

///////////////////////////////////////
//  Test Cases with Single Messages  //
///////////////////////////////////////

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanResolutionAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(stamped_msg.msg_.resolution(), scan_ptr->getScanResolution()) << "Resolution incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMinMaxScanAngleAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  const util::TenthOfDegree expected_max_scan_angle{
    stamped_msg.msg_.fromTheta() +
    stamped_msg.msg_.resolution() * (static_cast<int>(stamped_msg.msg_.measurements().size()) - 1)
  };

  EXPECT_EQ(stamped_msg.msg_.fromTheta(), scan_ptr->getMinScanAngle()) << "Min scan-angle incorrect in LaserScan";
  EXPECT_EQ(expected_max_scan_angle, scan_ptr->getMaxScanAngle()) << "Max scan-angle incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMeasurementsAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_THAT(scan_ptr->getMeasurements(), PointwiseDoubleEq(stamped_msg.msg_.measurements()));
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectIntensitiesAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_THAT(scan_ptr->getIntensities(), PointwiseDoubleEq(stamped_msg.msg_.intensities()));
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanCounterAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(stamped_msg.msg_.scanCounter(), scan_ptr->getScanCounter());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectTimestampAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->getTimestamp());
}

/////////////////////////////////////////
//  Test Cases with Multiple Messages  //
/////////////////////////////////////////

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingResolutions)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  CHANGE_MSG_PROPERTY(stamped_msgs[1].msg_, resolution, util::TenthOfDegree(10));
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
  auto stamped_msgs = createValidStampedMsgs(6);
  CHANGE_MSG_PROPERTY(stamped_msgs[1].msg_, fromTheta, util::TenthOfDegree(10));
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingScanCounters)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  CHANGE_MSG_PROPERTY(stamped_msgs[1].msg_, scanCounter, 1);
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, laserScanShouldContainAllScanInformationWhenBuildWithMultipleFrames)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  auto measurements_it = scan_ptr->getMeasurements().begin();
  auto intensities_it = scan_ptr->getIntensities().begin();
  for (const auto& stamped_msg : stamped_msgs)
  {
    ASSERT_LE(measurements_it + stamped_msg.msg_.measurements().size(), scan_ptr->getMeasurements().end());
    ASSERT_LE(intensities_it + stamped_msg.msg_.intensities().size(), scan_ptr->getIntensities().end());
    EXPECT_THAT(
        measurements_it,
        UnsafePointwiseDoubleItEq(stamped_msg.msg_.measurements().begin(), stamped_msg.msg_.measurements().end()));
    EXPECT_THAT(
        intensities_it,
        UnsafePointwiseDoubleItEq(stamped_msg.msg_.intensities().begin(), stamped_msg.msg_.intensities().end()));
    measurements_it += stamped_msg.msg_.measurements().size();
    intensities_it += stamped_msg.msg_.intensities().size();
  }
  EXPECT_EQ(measurements_it, scan_ptr->getMeasurements().end());
  EXPECT_EQ(intensities_it, scan_ptr->getIntensities().end());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectTimestampWhenBuildingWithMulitpleFrames)
{
  auto stamped_msgs = createValidStampedMsgs(3);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->getTimestamp());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMeasurementsOrderedByThetaAngle)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  CHANGE_MSG_VECTOR_PROPERTY(stamped_msgs.front().msg_, measurements, 0, 10.0);
  CHANGE_MSG_VECTOR_PROPERTY(
      stamped_msgs.back().msg_, measurements, stamped_msgs.back().msg_.measurements().size() - 1, 10.0);
  auto first_measurement = stamped_msgs.front().msg_.measurements().front();
  auto last_measurement = stamped_msgs.back().msg_.measurements().back();

  std::swap(stamped_msgs.front(), stamped_msgs.back());
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  ASSERT_EQ(first_measurement, scan_ptr->getMeasurements().front());
  ASSERT_EQ(last_measurement, scan_ptr->getMeasurements().back());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMinimalTimestamp)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  std::swap(stamped_msgs[0].stamp_, stamped_msgs[1].stamp_);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->getTimestamp());
}

TEST(LaserScanConversionsTest, laserScanShouldContainActiveZonesetOfLastMsg)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  CHANGE_MSG_PROPERTY(stamped_msgs[4].msg_, activeZoneset, 3);
  CHANGE_MSG_PROPERTY(stamped_msgs[5].msg_, activeZoneset, 4);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(4, scan_ptr->getActiveZoneset());
}

using Message = data_conversion_layer::monitoring_frame::Message;
using Stamped = data_conversion_layer::monitoring_frame::MessageStamped;
using Tenth = util::TenthOfDegree;

TEST(LaserScanConversionTest, conversionShouldIgnoreEmptyFramesForMonitoringFramesValidation)
{
  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<Stamped> stamped_msgs = {
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(2500)).measurements({}), 3),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(0)).measurements({}), 4),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(500)).measurements({}), 5),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(1318)).measurements({ 1., 2., 3. }), 6),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(1500)).measurements({}), 7),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(2000)).measurements({}), 8)
  };

  ASSERT_NO_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs));
}

TEST(LaserScanConversionsTest, conversionShouldIgnoreEmptyFramesForTimestampsComputation)
{
  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<Stamped> stamped_msgs = {
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(2500)).measurements({}), 3),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(0)).measurements({}), 4),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(500)).measurements({}), 5),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(1318)).measurements({ 1., 2., 3. }), 40000),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(1500)).measurements({}), 7),
    Stamped(createDefaultMsgBuilder().fromTheta(Tenth(2000)).measurements({}), 8)
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
