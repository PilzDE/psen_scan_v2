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

#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <geometry_msgs/Polygon.h>

#include "psen_scan_v2/zoneset_configuration_ros_conversion.h"

#include "psen_scan_v2_standalone/configuration/zoneset.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone::configuration;

namespace psen_scan_v2_test
{
TEST(ZoneSetROSConversionsTest, shouldCorrectlyConvertPolarCoordinates)
{
  geometry_msgs::Polygon poly1 = fromPolar({ 1000 }, util::TenthOfDegree(0) /*irrelevant here*/, 0 /*x_axis_rotation*/);
  EXPECT_DOUBLE_EQ(poly1.points.at(0).x, 1.0 /*m*/);
  EXPECT_DOUBLE_EQ(poly1.points.at(0).y, 0.0 /*m*/);

  geometry_msgs::Polygon poly2 =
      fromPolar({ 1000 }, util::TenthOfDegree(0) /*irrelevant here*/, -M_PI_2l /*x_axis_rotation*/);
  EXPECT_NEAR(poly2.points.at(0).x, 0.0 /*m*/, 1e-16);
  EXPECT_DOUBLE_EQ(poly2.points.at(0).y, 1.0 /*m*/);
}

TEST(ZoneSetROSConversionsTest, fromPolarWithSteps)
{
  geometry_msgs::Polygon poly1 = fromPolar({ 1000, 500 }, util::TenthOfDegree(900), 0 /*x_axis_rotation*/);
  EXPECT_DOUBLE_EQ(poly1.points.at(0).x, 1.0 /*m*/);
  EXPECT_DOUBLE_EQ(poly1.points.at(0).y, 0.0 /*m*/);
  EXPECT_NEAR(poly1.points.at(1).x, 0.0 /*m*/, 1e-16);
  EXPECT_DOUBLE_EQ(poly1.points.at(1).y, 0.5 /*m*/);

  geometry_msgs::Polygon poly2 = fromPolar({ 1000, 500 }, util::TenthOfDegree(900), -M_PI_2l /*x_axis_rotation*/);
  EXPECT_NEAR(poly1.points.at(1).x, 0.0 /*m*/, 1e-16);
  EXPECT_DOUBLE_EQ(poly2.points.at(0).y /*m*/, 1.0);
  EXPECT_DOUBLE_EQ(poly2.points.at(1).x /*m*/, -0.5);
  EXPECT_NEAR(poly2.points.at(1).y, 0.0 /*m*/, 1e-16);
}

TEST(ZoneSetROSConversionsTest, fromPolarWithActualOffset)
{
  geometry_msgs::Polygon poly2 =
      fromPolar({ 1000, 500 }, util::TenthOfDegree(1375), DEFAULT_X_AXIS_ROTATION /*x_axis_rotation*/);
  EXPECT_DOUBLE_EQ(poly2.points.at(1).x /*m*/, 0.5);
  EXPECT_NEAR(poly2.points.at(1).y, 0.0, 1e-16);
}

TEST(ZoneSetROSConversionsTest, ZoneSetToRosMsgCorrectPointConversion)
{
  ZoneSetStandalone zoneset;
  zoneset.safety1_ = { 1000, 2000, 2500 };
  zoneset.safety2_ = { 3000, 4000 };
  zoneset.safety3_ = { 5000, 6000 };
  zoneset.warn1_ = { 7000, 8000 };
  zoneset.warn2_ = { 9000, 10000 };
  zoneset.muting1_ = { 11000, 12000 };
  zoneset.muting2_ = { 13000, 14000 };
  zoneset.speed_range_ = ZoneSetSpeedRange(-5, 10);

  zoneset.resolution_ = DEFAULT_ZONESET_ANGLE_STEP;

  psen_scan_v2::ZoneSet zoneset_msg = toRosMsg(zoneset, "test_frame_id");

  // Use EXPECT_FLOAT_EQ since the msg has float internally
  EXPECT_FLOAT_EQ(zoneset_msg.safety1.points.at(0).x, 1.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.safety1.points.at(0).y, 1.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.safety1.points.at(1).x,
                  2.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.safety1.points.at(1).y,
                  2.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.safety1.points.at(2).x,
                  2.5 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad() * 2));
  EXPECT_FLOAT_EQ(zoneset_msg.safety1.points.at(2).y,
                  2.5 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad() * 2));

  EXPECT_FLOAT_EQ(zoneset_msg.safety2.points.at(0).x, 3.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.safety2.points.at(0).y, 3.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.safety2.points.at(1).x,
                  4.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.safety2.points.at(1).y,
                  4.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));

  EXPECT_FLOAT_EQ(zoneset_msg.safety3.points.at(0).x, 5.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.safety3.points.at(0).y, 5.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.safety3.points.at(1).x,
                  6.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.safety3.points.at(1).y,
                  6.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));

  EXPECT_FLOAT_EQ(zoneset_msg.warn1.points.at(0).x, 7.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.warn1.points.at(0).y, 7.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.warn1.points.at(1).x,
                  8.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.warn1.points.at(1).y,
                  8.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));

  EXPECT_FLOAT_EQ(zoneset_msg.warn2.points.at(0).x, 9.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.warn2.points.at(0).y, 9.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.warn2.points.at(1).x,
                  10.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.warn2.points.at(1).y,
                  10.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));

  EXPECT_FLOAT_EQ(zoneset_msg.muting1.points.at(0).x, 11.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.muting1.points.at(0).y, 11.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.muting1.points.at(1).x,
                  12.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.muting1.points.at(1).y,
                  12.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));

  EXPECT_FLOAT_EQ(zoneset_msg.muting2.points.at(0).x, 13.0 * std::cos(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.muting2.points.at(0).y, 13.0 * std::sin(-DEFAULT_X_AXIS_ROTATION));
  EXPECT_FLOAT_EQ(zoneset_msg.muting2.points.at(1).x,
                  14.0 * std::cos(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
  EXPECT_FLOAT_EQ(zoneset_msg.muting2.points.at(1).y,
                  14.0 * std::sin(-DEFAULT_X_AXIS_ROTATION + DEFAULT_ZONESET_ANGLE_STEP.toRad()));
}

TEST(ZoneSetROSConversionsTest, ZoneSetToRosMsgCorrectSpeedRangeConversion)
{
  ZoneSetStandalone zoneset;
  zoneset.speed_range_ = ZoneSetSpeedRange(-5, 10);

  psen_scan_v2::ZoneSet zoneset_msg = toRosMsg(zoneset, "test_frame_id");
  EXPECT_FLOAT_EQ(zoneset_msg.speed_lower, -5.0);
  EXPECT_FLOAT_EQ(zoneset_msg.speed_upper, 10.0);
}

TEST(ZoneSetROSConversionsTest, ZoneSetToRosMsgCorrectHeader)
{
  ZoneSetStandalone zoneset;
  zoneset.speed_range_ = ZoneSetSpeedRange(-5, 10);

  psen_scan_v2::ZoneSet zoneset_msg = toRosMsg(zoneset, "test_frame_id", ros::Time(1));
  EXPECT_EQ(zoneset_msg.header.frame_id, "test_frame_id");
  EXPECT_EQ(zoneset_msg.header.stamp, ros::Time(1));
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
