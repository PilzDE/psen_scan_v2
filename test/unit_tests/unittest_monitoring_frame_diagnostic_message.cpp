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

#include <gtest/gtest.h>

#include "psen_scan_v2/diagnostics.h"

using namespace psen_scan_v2;
namespace psen_scan_v2_test
{
TEST(MonitoringFrameDiagnosticMessageTest, shouldConstructMonitoringFrameDiagnosticMessageAsExpected)
{
  auto msg =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE0, monitoring_frame::diagnostic::ErrorLocation(4, 2));
  EXPECT_EQ(msg.getDiagnosticCode(), monitoring_frame::diagnostic::DiagnosticCode::CONF_ERR);
  EXPECT_EQ(msg.getErrorLocation().getByte(), static_cast<size_t>(4));
  EXPECT_EQ(msg.getErrorLocation().getBit(), static_cast<size_t>(2));
  EXPECT_EQ(msg.getScannerId(), ScannerId::SLAVE0);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeEqualOnSameInputData)
{
  auto msg0 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(3, 0));
  auto msg1 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(3, 0));
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnDifferentScannerId)
{
  auto msg0 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE0, monitoring_frame::diagnostic::ErrorLocation(3, 0));
  auto msg1 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(3, 0));
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnErrorByteLocation)
{
  auto msg0 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(0, 0));
  auto msg1 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(1, 0));
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnDifferentErrorBitLocation)
{
  auto msg0 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(1, 0));
  auto msg1 =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(1, 1));
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldOutputTheRightDiagnosticMessage)
{
  auto msg =
      monitoring_frame::diagnostic::Message(ScannerId::MASTER, monitoring_frame::diagnostic::ErrorLocation(3, 3));
  std::ostringstream os;
  os << msg;
  EXPECT_EQ(os.str(), "Device: Master - Display communication problem.");
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldOutputTheRightDiagnosticMessageWithBitandBytes)
{
  auto msg =
      monitoring_frame::diagnostic::Message(ScannerId::SLAVE1, monitoring_frame::diagnostic::ErrorLocation(2, 5));
  std::ostringstream os;
  os << msg;

  // Output with byte/bit information since error internal error is ambiguous
  EXPECT_EQ(os.str(), "Device: Slave1 - Internal error. (Byte:2 Bit:5)");
}
}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
