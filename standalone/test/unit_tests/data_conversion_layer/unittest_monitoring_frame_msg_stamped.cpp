// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static data_conversion_layer::monitoring_frame::Message
createMsg(const util::TenthOfDegree from_theta = util::TenthOfDegree{ 10 },
          const util::TenthOfDegree resolution = util::TenthOfDegree{ 90 },
          const uint32_t scan_counter = uint32_t{ 42 },
          const uint8_t active_zoneset = uint8_t{ 1 })
{
  const std::vector<double> measurements{ 1., 2., 3., 4.5, 5., 42. };
  const std::vector<double> intensities{ 0., 4., 3., 1007., 508., 14000. };
  const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message> diagnostic_messages{};

  return data_conversion_layer::monitoring_frame::Message(
      from_theta, resolution, scan_counter, active_zoneset, measurements, intensities, diagnostic_messages);
}

TEST(MonitoringFrameMsgStampedTest, testMsg)
{
  const auto expected_msg{ createMsg() };
  const data_conversion_layer::monitoring_frame::MessageStamped stamped_msg(expected_msg, int64_t{ 3 });
  EXPECT_EQ(expected_msg, stamped_msg.msg_);
}

TEST(MonitoringFrameMsgStampedTest, testStamp)
{
  const int64_t expected_stamp{ 3 };
  const auto msg{ createMsg() };
  const data_conversion_layer::monitoring_frame::MessageStamped stamped_msg(msg, expected_stamp);
  EXPECT_EQ(expected_stamp, stamped_msg.stamp_);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
