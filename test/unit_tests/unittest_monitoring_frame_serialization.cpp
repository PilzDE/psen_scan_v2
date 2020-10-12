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
  // Load testdata from dump
  UDPFrameTestDataWithoutIntensities test_data;
  DynamicSizeRawData serialized_monitoring_frame_message = serialize(test_data.msg_);

  EXPECT_EQ(test_data.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < test_data.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), test_data.hex_dump.at(i)) << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, testUDPFrameTestDataWithoutMeasurementsAndIntensitiesSuccess)
{
  // Load testdata from dump
  UDPFrameTestDataWithoutMeasurementsAndIntensities test_data;
  DynamicSizeRawData serialized_monitoring_frame_message = serialize(test_data.msg_);

  EXPECT_EQ(test_data.hex_dump.size(), serialized_monitoring_frame_message.size());

  for (size_t i = 0; i < test_data.hex_dump.size(); i++)
  {
    EXPECT_EQ((uint8_t)serialized_monitoring_frame_message.at(i), test_data.hex_dump.at(i)) << " index " << i;
  }
}

TEST(MonitoringFrameSerializationTest, testSerializationInvariance)
{
  UDPFrameTestDataWithoutIntensities test_data;
  DynamicSizeRawData raw = serialize(test_data.msg_);

  MonitoringFrameMsg deserialized_msg = deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(test_data.msg_, deserialized_msg);
}

TEST(MonitoringFrameSerializationTest, testSerializationInvariance2)
{
  MonitoringFrameMsg msg(TenthOfDegree(25), TenthOfDegree(1), 456, { 10, 20, 30, 40 });

  DynamicSizeRawData raw = serialize(msg);

  MonitoringFrameMsg deserialized_msg = deserialize(convertToMaxSizeRawData(raw), raw.size());

  EXPECT_EQ(msg, deserialized_msg);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}