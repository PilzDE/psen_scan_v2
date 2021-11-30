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
#include <string>

#include <gtest/gtest.h>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"

#include "psen_scan_v2_standalone/data_conversion_layer/istring_stream_builder.h"
#include "psen_scan_v2_standalone/communication_layer/udp_frame_dumps.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"

#include "psen_scan_v2_standalone/util/gtest_expectations.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2_standalone_test;
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

static const std::string ADDITIONAL_FIELD_MISSING_TEXT = " not set! (Contact PILZ support if the error persists.)";

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetScanCounter)
{
  data_conversion_layer::monitoring_frame::Message msg{};
  EXPECT_THROW_AND_WHAT(msg.scanCounter(),
                        data_conversion_layer::monitoring_frame::AdditionalFieldMissing,
                        ("Scan counter" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetMeasurements)
{
  data_conversion_layer::monitoring_frame::Message msg{};
  EXPECT_THROW_AND_WHAT(msg.measurements(),
                        data_conversion_layer::monitoring_frame::AdditionalFieldMissing,
                        ("Measurements" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetIntensities)
{
  data_conversion_layer::monitoring_frame::Message msg{};
  EXPECT_THROW_AND_WHAT(msg.intensities(),
                        data_conversion_layer::monitoring_frame::AdditionalFieldMissing,
                        ("Intensities" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetActiveZoneset)
{
  data_conversion_layer::monitoring_frame::Message msg{};
  EXPECT_THROW_AND_WHAT(msg.activeZoneset(),
                        data_conversion_layer::monitoring_frame::AdditionalFieldMissing,
                        ("Active zoneset" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgTest, shouldThrowAdditionalFieldMissingWhenTryingToGetUnsetDiagnosticMessages)
{
  data_conversion_layer::monitoring_frame::Message msg{};
  EXPECT_THROW_AND_WHAT(msg.diagnosticMessages(),
                        data_conversion_layer::monitoring_frame::AdditionalFieldMissing,
                        ("Diagnostic messages" + ADDITIONAL_FIELD_MISSING_TEXT).c_str());
}

TEST(MonitoringFrameMsgPrintTest, testPrintMessageSuccess)
{
  auto msg = data_conversion_layer::monitoring_frame::MessageBuilder()
                 .fromTheta(util::TenthOfDegree(1234))
                 .resolution(util::TenthOfDegree(56))
                 .scanCounter(78)
                 .activeZoneset(2)
                 .measurements({ 45, 44, 43, 42 })
                 .build();

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

}  // namespace psen_scan_v2

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
