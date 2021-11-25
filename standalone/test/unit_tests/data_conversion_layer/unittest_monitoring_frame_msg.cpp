// Copyright (c) 2019-2021 Pilz GmbH & Co. KG
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

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"

#include "psen_scan_v2_standalone/data_conversion_layer/istring_stream_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"
#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
using namespace data_conversion_layer::monitoring_frame;

const util::TenthOfDegree DEFAULT_FROM_THETA(100);
const util::TenthOfDegree DEFAULT_RESOLUTION(10);
const uint32_t DEFAULT_SCAN_COUNTER(42);
const uint8_t DEFAULT_ACTIVE_ZONESET(0);
const IOState DEFAULT_IO_STATE({ PinState(1, "zone1", true) },
                               { PinState(2, "zone1", true) },
                               { PinState(1, "zone1", false) },
                               { PinState(4, "OSST", false) },
                               { PinState(5, "", true) });
const std::vector<double> DEFAULT_MEASUREMENTS{ 1, 2, 3 };
const std::vector<double> DEFAULT_INTENSITIES{ 10, 20, 30 };
const std::vector<diagnostic::Message> DEFAULT_DIAGNOSTIC_MSGS{ diagnostic::Message(configuration::ScannerId::master,
                                                                                    diagnostic::ErrorLocation(5, 3)) };

static Message
createMonitoringFrameMsg(const util::TenthOfDegree& from_theta = DEFAULT_FROM_THETA,
                         const util::TenthOfDegree& resolution = DEFAULT_RESOLUTION,
                         const uint32_t scan_counter = DEFAULT_SCAN_COUNTER,
                         const uint8_t active_zoneset = DEFAULT_ACTIVE_ZONESET,
                         const IOState& io_state = DEFAULT_IO_STATE,
                         const std::vector<double>& measurements = DEFAULT_MEASUREMENTS,
                         const std::vector<double>& intensities = DEFAULT_INTENSITIES,
                         const std::vector<diagnostic::Message>& diagnostic_messages = DEFAULT_DIAGNOSTIC_MSGS)
{
  return Message(
      from_theta, resolution, scan_counter, active_zoneset, io_state, measurements, intensities, diagnostic_messages);
}

static Message createDefaultMsgAndSetIntensities(const std::vector<double>& intensities)
{
  return createMonitoringFrameMsg(DEFAULT_FROM_THETA,
                                  DEFAULT_RESOLUTION,
                                  DEFAULT_SCAN_COUNTER,
                                  DEFAULT_ACTIVE_ZONESET,
                                  DEFAULT_IO_STATE,
                                  DEFAULT_MEASUREMENTS,
                                  intensities);
}

static Message createDefaultMsgAndSetMeasurements(const std::vector<double>& measurements)
{
  return createMonitoringFrameMsg(DEFAULT_FROM_THETA,
                                  DEFAULT_RESOLUTION,
                                  DEFAULT_SCAN_COUNTER,
                                  DEFAULT_ACTIVE_ZONESET,
                                  DEFAULT_IO_STATE,
                                  measurements);
}

static Message createDefaultMsgAndSetFromTheta(util::TenthOfDegree from_theta)
{
  return createMonitoringFrameMsg(from_theta);
}

static Message createDefaultMsgAndSetResolution(util::TenthOfDegree resolution)
{
  return createMonitoringFrameMsg(DEFAULT_FROM_THETA, resolution);
}

static Message createDefaultMsgAndSetScanCounter(uint32_t scan_counter)
{
  return createMonitoringFrameMsg(DEFAULT_FROM_THETA, DEFAULT_RESOLUTION, scan_counter);
}

static Message createDefaultMsgAndSetActiveZone(uint8_t active_zone)
{
  return createMonitoringFrameMsg(DEFAULT_FROM_THETA, DEFAULT_RESOLUTION, DEFAULT_SCAN_COUNTER, active_zone);
}

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
  const Message msg0(util::TenthOfDegree(100), util::TenthOfDegree(10), 42, 1, DEFAULT_IO_STATE, { 1, 2, 3 });
  const Message msg1(util::TenthOfDegree(100), util::TenthOfDegree(10), 42, 1, DEFAULT_IO_STATE, { 1, 2, 3 });
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualIntensitiesSucces)
{
  auto msg0 = createMonitoringFrameMsg();
  auto msg1 = createMonitoringFrameMsg();
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualEmptySuccess)
{
  const Message msg0(util::TenthOfDegree(100), util::TenthOfDegree(10), 42, 0, DEFAULT_IO_STATE, {});
  const Message msg1(util::TenthOfDegree(100), util::TenthOfDegree(10), 42, 0, DEFAULT_IO_STATE, {});
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareEqualIntensitiesEmptySucces)
{
  const Message msg0(util::TenthOfDegree(100), util::TenthOfDegree(10), 42, 0, DEFAULT_IO_STATE, {}, {}, {});
  const Message msg1(util::TenthOfDegree(100), util::TenthOfDegree(10), 42, 0, DEFAULT_IO_STATE, {}, {}, {});
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareMeasurementsNotEqual)
{
  const auto msg0 = createDefaultMsgAndSetMeasurements({ 1, 42, 3 });
  const auto msg1 = createDefaultMsgAndSetMeasurements({ 1, 2, 3 });
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareIntensitiesNotEqual)
{
  const auto msg0 = createDefaultMsgAndSetIntensities({ 10, 42, 30 });
  const auto msg1 = createDefaultMsgAndSetIntensities({ 10, 20, 30 });
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareFromThetaNotEqual)
{
  const auto msg0 = createDefaultMsgAndSetFromTheta(util::TenthOfDegree(42));
  const auto msg1 = createDefaultMsgAndSetFromTheta(util::TenthOfDegree(100));
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareResolutionNotEqual)
{
  const auto msg0 = createDefaultMsgAndSetResolution(util::TenthOfDegree(42));
  const auto msg1 = createDefaultMsgAndSetResolution(util::TenthOfDegree(10));
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, shouldCompareToFalseOnMessagesWithDifferentCounter)
{
  const auto msg0 = createDefaultMsgAndSetScanCounter(42);
  const auto msg1 = createDefaultMsgAndSetScanCounter(1);
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgEqualityTest, shouldCompareToFalseOnMessagesWithDifferentActiveZoneset)
{
  const auto msg0 = createDefaultMsgAndSetActiveZone(0);
  const auto msg1 = createDefaultMsgAndSetActiveZone(42);
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgTest, shouldThrowMissingScanCounterErrorWhenScanCounterWasNeverSet)
{
  Message msg{};
  EXPECT_THROW(msg.scanCounter(), ScanCounterMissing);
}

TEST(MonitoringFrameMsgEqualityTest, testCompareNotEqualEmpty)
{
  const Message msg0(util::TenthOfDegree(100), util::TenthOfDegree(42), 42, 0, DEFAULT_IO_STATE, {}, {}, {});
  const Message msg1(util::TenthOfDegree(100), util::TenthOfDegree(42), 0, 0, DEFAULT_IO_STATE, {}, {}, {});
  EXPECT_NE(msg0, msg1);
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccess)
{
  Message msg(util::TenthOfDegree(1234), util::TenthOfDegree(56), 78, 2, DEFAULT_IO_STATE, { 45, 44, 43, 42 });

// For compatibility with different ubuntu versions (resp. fmt), we need to take account of changes in
// the default formatting of floating point numbers
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "active_zoneset = 2, measurements = {45.0, 44.0, 43.0, 42.0}, intensities = {}, diagnostics = {})");
#else
  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "active_zoneset = 2, measurements = {45, 44, 43, 42}, intensities = {}, diagnostics = {})");
#endif
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
