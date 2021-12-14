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

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"

using namespace psen_scan_v2_standalone;
namespace psen_scan_v2_standalone_test
{
TEST(MonitoringFrameDiagnosticMessageTest, shouldConstructMonitoringFrameDiagnosticMessageAsExpected)
{
  auto msg = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave0, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(4, 3));
  EXPECT_EQ(msg.getDiagnosticCode(), data_conversion_layer::monitoring_frame::diagnostic::ErrorType::conf_err);
  EXPECT_EQ(msg.getErrorLocation().byte(), static_cast<size_t>(4));
  EXPECT_EQ(msg.getErrorLocation().bit(), static_cast<size_t>(3));
  EXPECT_EQ(msg.getScannerId(), configuration::ScannerId::slave0);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeEqualOnSameInputData)
{
  auto msg0 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(3, 0));
  auto msg1 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(3, 0));
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnDifferentScannerId)
{
  auto msg0 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave0, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(3, 0));
  auto msg1 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(3, 0));
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnErrorByteLocation)
{
  auto msg0 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(0, 0));
  auto msg1 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(1, 0));
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnDifferentErrorBitLocation)
{
  auto msg0 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(1, 0));
  auto msg1 = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(1, 1));
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldOutputTheRightDiagnosticMessage)
{
  auto msg = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::master, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(3, 3));
  std::ostringstream os;
  os << msg;
  EXPECT_EQ(os.str(), "Device: Master - Display communication problem.");
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldOutputTheRightDiagnosticMessageWithBitandBytes)
{
  auto msg = data_conversion_layer::monitoring_frame::diagnostic::Message(
      configuration::ScannerId::slave1, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(2, 5));
  std::ostringstream os;
  os << msg;

  // Output with byte/bit information since error internal error is ambiguous
  EXPECT_EQ(os.str(), "Device: Slave1 - Internal error. (Byte:2 Bit:5)");
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
