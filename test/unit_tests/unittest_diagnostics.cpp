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
  MonitoringFrameDiagnosticMessage msg = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE0, 4, 2);
  EXPECT_EQ(msg.getDiagnosticCode(), DiagnosticCode::CONF_ERR);
  EXPECT_EQ(msg.getErrorByteLocation(), 4);
  EXPECT_EQ(msg.getErrorBitLocation(), 2);
  EXPECT_EQ(msg.getScannerId(), ScannerId::SLAVE0);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeEqualOnSameInputData)
{
  MonitoringFrameDiagnosticMessage msg0 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 3, 0);
  MonitoringFrameDiagnosticMessage msg1 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 3, 0);
  EXPECT_EQ(msg0, msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnDifferentScannerId)
{
  MonitoringFrameDiagnosticMessage msg0 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE0, 3, 0);
  MonitoringFrameDiagnosticMessage msg1 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 3, 0);
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnErrorByteLocation)
{
  MonitoringFrameDiagnosticMessage msg0 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 0, 0);
  MonitoringFrameDiagnosticMessage msg1 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 1, 0);
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldBeNotEqualOnDifferentErrorBitLocation)
{
  MonitoringFrameDiagnosticMessage msg0 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 1, 0);
  MonitoringFrameDiagnosticMessage msg1 = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 1, 1);
  EXPECT_FALSE(msg0 == msg1);
}

TEST(MonitoringFrameDiagnosticMessageTest, shouldOutputTheRightDiagnosticMessage)
{
  MonitoringFrameDiagnosticMessage msg = MonitoringFrameDiagnosticMessage(ScannerId::SLAVE1, 1, 0);
  ;
  std::ostringstream os;
  os << msg;
  EXPECT_EQ(os.str(), "Device: Slave1 - OSSD2_OVERCUR (Byte:1 Bit:0)");
}
}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
