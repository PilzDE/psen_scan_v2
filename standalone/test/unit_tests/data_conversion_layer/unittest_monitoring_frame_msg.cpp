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

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

#include "psen_scan_v2_standalone/util/gtest_expectations.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
using namespace data_conversion_layer::monitoring_frame;

static const std::string ADDITIONAL_FIELD_MISSING_TEXT = " not set! (Contact PILZ support if the error persists.)";

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetScanCounter)
{
  EXPECT_THROW_AND_WHAT(
      Message().scanCounter(), AdditionalFieldMissing, ("Scan counter" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetMeasurements)
{
  EXPECT_THROW_AND_WHAT(
      Message().measurements(), AdditionalFieldMissing, ("Measurements" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetIntensities)
{
  EXPECT_THROW_AND_WHAT(
      Message().intensities(), AdditionalFieldMissing, ("Intensities" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetActiveZoneset)
{
  EXPECT_THROW_AND_WHAT(
      Message().activeZoneset(), AdditionalFieldMissing, ("Active zoneset" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetDiagnosticMessages)
{
  EXPECT_THROW_AND_WHAT(Message().diagnosticMessages(),
                        AdditionalFieldMissing,
                        ("Diagnostic messages" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectStateOfScanCounter)
{
  EXPECT_FALSE(MessageBuilder().build().hasScanCounterField());
  EXPECT_TRUE(MessageBuilder().scanCounter(2).build().hasScanCounterField());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectStateOfActiveZoneset)
{
  EXPECT_FALSE(MessageBuilder().build().hasActiveZonesetField());
  EXPECT_TRUE(MessageBuilder().activeZoneset(2).build().hasActiveZonesetField());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectStateOfMeasurements)
{
  EXPECT_FALSE(MessageBuilder().build().hasMeasurementsField());
  EXPECT_TRUE(MessageBuilder().measurements({}).build().hasMeasurementsField());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectStateOfIntensities)
{
  EXPECT_FALSE(MessageBuilder().build().hasIntensitiesField());
  EXPECT_TRUE(MessageBuilder().intensities({}).build().hasIntensitiesField());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectStateOfDiagnosticMessages)
{
  EXPECT_FALSE(MessageBuilder().build().hasDiagnosticMessagesField());
  EXPECT_TRUE(MessageBuilder().diagnosticMessages({}).build().hasDiagnosticMessagesField());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectScannerId)
{
  const auto scanner_id{ configuration::ScannerId::slave0 };
  EXPECT_EQ(scanner_id, MessageBuilder().scannerId(scanner_id).build().scannerId());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectFromTheta)
{
  const auto from_theta{ util::TenthOfDegree(50) };
  EXPECT_EQ(from_theta, MessageBuilder().fromTheta(from_theta).build().fromTheta());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectResolution)
{
  const auto resolution{ util::TenthOfDegree(10) };
  EXPECT_EQ(resolution, MessageBuilder().resolution(resolution).build().resolution());
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectScanCounter)
{
  const uint32_t expected_scan_counter{ 42 };
  uint32_t scan_counter{ 0 };
  ASSERT_NO_THROW(scan_counter = MessageBuilder().scanCounter(expected_scan_counter).build().scanCounter());
  EXPECT_EQ(expected_scan_counter, scan_counter);
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectActiveZoneset)
{
  const uint8_t expected_active_zoneset{ 3 };
  uint8_t active_zoneset{ 0 };
  ASSERT_NO_THROW(active_zoneset = MessageBuilder().activeZoneset(expected_active_zoneset).build().activeZoneset());
  EXPECT_EQ(expected_active_zoneset, active_zoneset);
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectMeasurements)
{
  const std::vector<double> expected_measurements{ { 2.1, 1.3 } };
  std::vector<double> measurements;
  ASSERT_NO_THROW(measurements = MessageBuilder().measurements(expected_measurements).build().measurements());
  EXPECT_EQ(expected_measurements, measurements);
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectIntensities)
{
  const std::vector<double> expected_intensities{ { 2.1, 1.3 } };
  std::vector<double> intensities;
  ASSERT_NO_THROW(intensities = MessageBuilder().intensities(expected_intensities).build().intensities());
  EXPECT_EQ(expected_intensities, intensities);
}

TEST(MonitoringFrameMsgTest, shouldReturnCorrectDiagnosticMessages)
{
  const std::vector<diagnostic::Message> expected_diagnostic_messages{ { diagnostic::Message(
      configuration::ScannerId::master, diagnostic::ErrorLocation(1, 7)) } };
  std::vector<diagnostic::Message> diagnostic_messages;
  ASSERT_NO_THROW(diagnostic_messages =
                      MessageBuilder().diagnosticMessages(expected_diagnostic_messages).build().diagnosticMessages());
  EXPECT_EQ(expected_diagnostic_messages, diagnostic_messages);
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccessWithAdditionalFields)
{
  auto msg = MessageBuilder()
                 .fromTheta(util::TenthOfDegree(1234))
                 .resolution(util::TenthOfDegree(56))
                 .scanCounter(78)
                 .activeZoneset(2)
                 .measurements({ 45, 44, 43, 42 })
                 .intensities({ 1 })
                 .diagnosticMessages({})
                 .build();

// For compatibility with different ubuntu versions (resp. fmt), we need to take account of changes in
// the default formatting of floating point numbers
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "active_zoneset = 2, measurements = {45.0, 44.0, 43.0, 42.0}, intensities = {1.0}, diagnostics = {})");
#else
  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = 78, "
            "active_zoneset = 2, measurements = {45, 44, 43, 42}, intensities = {1}, diagnostics = {})");
#endif
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccessWithoutAdditionalFields)
{
  auto msg = MessageBuilder().fromTheta(util::TenthOfDegree(1234)).resolution(util::TenthOfDegree(56)).build();

  EXPECT_EQ(fmt::format("{}", msg),
            "monitoring_frame::Message(fromTheta = 123.4 deg, resolution = 5.6 deg, scanCounter = _, "
            "active_zoneset = _, measurements = _, intensities = _, diagnostics = _)");
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
