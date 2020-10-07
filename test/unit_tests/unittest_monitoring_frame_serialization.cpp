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
#include "psen_scan_v2/raw_data_array_conversion.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(MonitoringFrameSerializationTest, testUDPFrameTestDataWithoutIntensitiesSuccess)
{
  UDPFrameTestDataWithoutIntensities test_data;
  MaxSizeRawData raw_frame_data_ = convertToMaxSizeRawData(test_data.hex_dump);

  MonitoringFrameMsg msg = MonitoringFrameMsg::deserialize(raw_frame_data_, test_data.hex_dump.size());
  DynamicSizeRawData msg_raw = serialize(msg);

  EXPECT_EQ(test_data.hex_dump.size(), msg_raw.size());

  for (size_t i = 0; i < test_data.hex_dump.size(); i++)
  {
    uint8_t hex_dump_byte = test_data.hex_dump.at(i);
    uint8_t msg_raw_byte = msg_raw.at(i);
    EXPECT_EQ(msg_raw_byte, hex_dump_byte);
  }
}
}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}