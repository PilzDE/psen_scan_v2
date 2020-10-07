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

#include "psen_scan_v2/monitoring_frame_serialization.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/udp_frame_dumps.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(MonitoringFrameSerializationTest, testUDPFrameTestDataWithoutIntensitiesSuccess)
{
  MonitoringFrameMsg msg;
  DynamicSizeRawData msg_raw = serialize(msg);
  EXPECT_EQ(msg_raw.at(0), 1);
}
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}