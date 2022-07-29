// Copyright (c) 2020-2022 Pilz GmbH & Co. KG
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
#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/data_conversion_layer/laserscan_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data_helper.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

#define ADD_OFFSET_TO_SCALAR_MSG_PROPERTY(msg, property, offset)                                                       \
  msg = MessageBuilder()                                                                                               \
            .scannerId(msg.scannerId())                                                                                \
            .fromTheta(msg.fromTheta())                                                                                \
            .resolution(msg.resolution())                                                                              \
            .scanCounter(msg.scanCounter())                                                                            \
            .activeZoneset(msg.activeZoneset())                                                                        \
            .measurements(msg.measurements())                                                                          \
            .intensities(msg.intensities())                                                                            \
            .property(msg.property() + offset)

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
using data_conversion_layer::monitoring_frame::MessageBuilder;
using data_conversion_layer::monitoring_frame::MessageStamped;
using data_conversion_layer::monitoring_frame::io::PinData;

static const int64_t DEFAULT_TIMESTAMP{ 4500000 };
static const int64_t EXPECTED_TIMESTAMP_AFTER_CONVERSION{ 4400000 };  // 4500000 - ((1.2/360) * 30*10^6)

static size_t lastVectorIndex(const std::vector<double>& vec)
{
  return vec.size() - 1;
}

/**
 * @brief Replaces a monitoring_frame::Message by a copy which has an offset added to one of its measurements.
 * @return The new value of the changed measurement.
 */
static double addOffsetToMsgMeasurement(data_conversion_layer::monitoring_frame::Message& msg,
                                        std::size_t index,
                                        const double& offset)
{
  auto measurements_copy = msg.measurements();
  measurements_copy.at(index) += offset;
  msg = MessageBuilder()
            .scannerId(msg.scannerId())
            .fromTheta(msg.fromTheta())
            .resolution(msg.resolution())
            .scanCounter(msg.scanCounter())
            .activeZoneset(msg.activeZoneset())
            .iOPinData(msg.iOPinData())
            .measurements(measurements_copy)
            .intensities(msg.intensities());
  return measurements_copy.at(index);
}

static MessageBuilder createDefaultMsgBuilder()
{
  return MessageBuilder()
      .fromTheta(util::TenthOfDegree{ 10 })
      .resolution(util::TenthOfDegree{ 2 })
      .scanCounter(42)
      .activeZoneset(0)
      .iOPinData(createPinData())
      .measurements({ 1., 2., 3., 4.5, 5., 42., .4 })
      .intensities({ 0., 4., 3., 1007., 508., 14000., .4 });
}

static MessageStamped createDefaultStampedMsg(const int64_t timestamp = DEFAULT_TIMESTAMP, uint32_t msg_nr = 0)
{
  return MessageStamped(createDefaultMsgBuilder().iOPinData(
                            createPinData({ msg_nr % 8, 0, 0, 0, 0, 0, 0, 0 }, { 7 - (msg_nr % 8), 0, 0, 0 })),
                        timestamp);
}

static void addStampedMsg(std::vector<MessageStamped>& msgs, uint32_t msg_nr = 0)
{
  if (msgs.empty())
  {
    msgs.push_back(createDefaultStampedMsg(DEFAULT_TIMESTAMP, msg_nr));
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
    addStampedMsg(msgs, i);
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

  EXPECT_EQ(stamped_msg.msg_.resolution(), scan_ptr->scanResolution()) << "Resolution incorrect in LaserScan";
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

  EXPECT_EQ(stamped_msg.msg_.fromTheta(), scan_ptr->minScanAngle()) << "Min scan-angle incorrect in LaserScan";
  EXPECT_EQ(expected_max_scan_angle, scan_ptr->maxScanAngle()) << "Max scan-angle incorrect in LaserScan";
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectMeasurementsAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_THAT(scan_ptr->measurements(), PointwiseDoubleEq(stamped_msg.msg_.measurements()));
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectIntensitiesAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_THAT(scan_ptr->intensities(), PointwiseDoubleEq(stamped_msg.msg_.intensities()));
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectScanCounterAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(stamped_msg.msg_.scanCounter(), scan_ptr->scanCounter());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectTimestampAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->timestamp());
}

MATCHER_P(IOStateFromStampedMsg, stamped_msg, "")
{
  return ExplainMatchResult(IOStateEq(IOState(stamped_msg.msg_.iOPinData(), stamped_msg.stamp_)), arg, result_listener);
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectIOStateAfterConversion)
{
  const auto stamped_msg{ createDefaultStampedMsg() };

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan({ stamped_msg }) }););

  ASSERT_EQ(scan_ptr->ioStates().size(), 1u);
  EXPECT_THAT(scan_ptr->ioStates().at(0), IOStateFromStampedMsg(stamped_msg));
}

/////////////////////////////////////////
//  Test Cases with Multiple Messages  //
/////////////////////////////////////////

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingResolutions)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  ADD_OFFSET_TO_SCALAR_MSG_PROPERTY(stamped_msgs[1].msg_, resolution, util::TenthOfDegree(10));
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
  ADD_OFFSET_TO_SCALAR_MSG_PROPERTY(stamped_msgs[1].msg_, fromTheta, util::TenthOfDegree(10));
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, shouldThrowProtocolErrorOnMismatchingScanCounters)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  ADD_OFFSET_TO_SCALAR_MSG_PROPERTY(stamped_msgs[1].msg_, scanCounter, 1);
  ASSERT_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs),
               data_conversion_layer::ScannerProtocolViolationError);
}

TEST(LaserScanConversionsTest, laserScanShouldContainAllScanInformationWhenBuildWithMultipleFrames)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  EXPECT_EQ(stamped_msgs.size() * stamped_msgs[0].msg_.measurements().size(), scan_ptr->measurements().size());
  EXPECT_EQ(stamped_msgs.size() * stamped_msgs[0].msg_.intensities().size(), scan_ptr->intensities().size());
}

TEST(LaserScanConversionsTest, laserScanShouldContainCorrectTimestampWhenBuildingWithMulitpleFrames)
{
  auto stamped_msgs = createValidStampedMsgs(3);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->timestamp());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMeasurementsOrderedByThetaAngle)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  auto& first_msg = stamped_msgs.front();
  auto& last_msg = stamped_msgs.back();
  auto first_measurement = addOffsetToMsgMeasurement(first_msg.msg_, 0, 10.0);
  auto last_measurement = addOffsetToMsgMeasurement(last_msg.msg_, lastVectorIndex(last_msg.msg_.measurements()), 10.0);

  std::swap(first_msg, last_msg);
  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  EXPECT_EQ(first_measurement, scan_ptr->measurements().front());
  EXPECT_EQ(last_measurement, scan_ptr->measurements().back());
}

TEST(LaserScanConversionsTest, laserScanShouldContainMinimalTimestamp)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  std::swap(stamped_msgs[0].stamp_, stamped_msgs[1].stamp_);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(EXPECTED_TIMESTAMP_AFTER_CONVERSION, scan_ptr->timestamp());
}

TEST(LaserScanConversionsTest, laserScanShouldContainAllIOStates)
{
  const size_t msg_count = 6;
  const auto stamped_msgs = createValidStampedMsgs(msg_count);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  ASSERT_EQ(scan_ptr->ioStates().size(), msg_count);
  for (size_t i = 0; i < msg_count; i++)
  {
    EXPECT_THAT(scan_ptr->ioStates().at(i), IOStateFromStampedMsg(stamped_msgs.at(i)));
  }
}

TEST(LaserScanConversionsTest, laserScanShouldContainAllIOStatesInCorrectOrder)
{
  const size_t msg_count = 3;
  auto stamped_msgs = createValidStampedMsgs(msg_count);
  std::swap(stamped_msgs.back(), stamped_msgs.front());

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  // Issue #320: Due to bugfix the test have to change, Now we expect the ioStates as reception order.
  EXPECT_THAT(scan_ptr->ioStates().at(0), IOStateFromStampedMsg(stamped_msgs.at(0)));
  EXPECT_THAT(scan_ptr->ioStates().at(1), IOStateFromStampedMsg(stamped_msgs.at(1)));
  EXPECT_THAT(scan_ptr->ioStates().at(2), IOStateFromStampedMsg(stamped_msgs.at(2)));
}

TEST(LaserScanConversionsTest, laserScanShouldContainActiveZonesetOfLastMsg)
{
  auto stamped_msgs = createValidStampedMsgs(6);
  ADD_OFFSET_TO_SCALAR_MSG_PROPERTY(stamped_msgs[4].msg_, activeZoneset, 3);
  ADD_OFFSET_TO_SCALAR_MSG_PROPERTY(stamped_msgs[5].msg_, activeZoneset, 4);

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););
  EXPECT_EQ(4, scan_ptr->activeZoneset());
}

TEST(LaserScanConversionTest, conversionShouldIgnoreEmptyFramesWhenCheckingIfFromThetasFitTogether)
{
  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<MessageStamped> stamped_msgs = {
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(2500)).measurements({}), 3),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(0)).measurements({}), 4),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(500)).measurements({}), 5),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(1318)).measurements({ 1., 2., 3. }), 6),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(1500)).measurements({}), 7),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(2000)).measurements({}), 8)
  };

  EXPECT_NO_THROW(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs));
}

TEST(LaserScanConversionsTest, conversionShouldIgnoreEmptyFramesForTimestampsComputation)
{
  // The following from_theta's are a real example from wireshark.
  // (angle_start:=-0.1, angle_end:=0.1)
  std::vector<MessageStamped> stamped_msgs = {
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(2500)).measurements({}), 3),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(0)).measurements({}), 4),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(500)).measurements({}), 5),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(1318)).measurements({ 1., 2., 3. }), 40000),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(1500)).measurements({}), 7),
    MessageStamped(createDefaultMsgBuilder().fromTheta(util::TenthOfDegree(2000)).measurements({}), 8)
  };
  const int64_t expected_stamp{ 6667 };  // 40000 - ((0.4/360) * 30*10^6)

  std::unique_ptr<LaserScan> scan_ptr;
  ASSERT_NO_THROW(
      scan_ptr.reset(new LaserScan{ data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs) }););

  EXPECT_EQ(expected_stamp, scan_ptr->timestamp());
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
