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

#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>

#include "psen_scan_v2/ZoneSet.h"
#include "psen_scan_v2/zoneset_to_marker_conversion.h"

namespace psen_scan_v2_test
{
using namespace psen_scan_v2;

///////////////
//  Matcher  //
///////////////

MATCHER_P(ContainsMarkerWithNamespace, ns, "")
{
  return std::find_if(arg.begin(), arg.end(), [&](const auto& marker) { return marker.ns == ns; }) != arg.end();
}

MATCHER(HasAtLeastOneTriangle, "")
{
  return arg.points.size() >= 3;
}

using ::testing::DoubleEq;
using ::testing::Matches;

MATCHER(PointEQZero, "")
{
  return Matches(DoubleEq(0.0))(arg.x) && Matches(DoubleEq(0.0))(arg.y) && Matches(DoubleEq(0.0))(arg.z);
}

MATCHER_P(PointEQxyOf, point, "")
{
  return Matches(DoubleEq(point.x))(arg.x) && Matches(DoubleEq(point.y))(arg.y) && Matches(DoubleEq(0.0))(arg.z);
}

MATCHER_P(IsTriangleListOfArcSegment, points, "")
{
  auto arc_points_it = points.begin();
  auto triangle_points_it = arg.begin();
  while (triangle_points_it + 2 < arg.end() && arc_points_it + 1 < points.end())
  {
    if (!Matches(PointEQZero())(*triangle_points_it))
    {
      *result_listener << "First triangle point " << *triangle_points_it << " doesn't match (0, 0, 0).";
      return false;
    }
    if (!Matches(PointEQxyOf(*arc_points_it))(*++triangle_points_it))
    {
      *result_listener << "Second triangle point " << *triangle_points_it << " doesn't match " << *arc_points_it << ".";
      return false;
    }
    if (!Matches(PointEQxyOf(*++arc_points_it))(*++triangle_points_it))
    {
      *result_listener << "Third triangle point " << *triangle_points_it << " doesn't match " << *arc_points_it << ".";
      return false;
    }
    // Move to next triangle but reuse the second arc_point as first one of next triangle.
    triangle_points_it++;
  }
  if (triangle_points_it != arg.end() || arc_points_it + 1 != points.end())
  {
    *result_listener << "Length " << arg.size() << " of triangle points list doesn't match.";
    return false;
  }
  return true;
}

MATCHER_P4(AllElementsMatchColorRGBA, r, g, b, a, "")
{
  return std::all_of(arg.begin(), arg.end(), [&](const auto& color) {
    return color.r == r && color.g == g && color.b == b && color.a == a;
  });
}

/////////////////
//  Constants  //
/////////////////

static const std::vector<std::string> POLYGON_NAMES{ "safety1", "safety2", "safety3", "warn1",
                                                     "warn2",   "muting1", "muting2" };

/////////////////
//  Functions  //
/////////////////

static std::map<std::string, const geometry_msgs::Polygon&> generateNameToPolygonMap(const ZoneSet& zone_set)
{
  return std::map<std::string, const geometry_msgs::Polygon&>{
    { "safety1", zone_set.safety1 }, { "safety2", zone_set.safety2 }, { "safety3", zone_set.safety3 },
    { "warn1", zone_set.warn1 },     { "warn2", zone_set.warn2 },     { "muting1", zone_set.muting1 },
    { "muting2", zone_set.muting2 }
  };
}

static geometry_msgs::Point32 createPoint32(const float& x, const float& y, const float& z)
{
  geometry_msgs::Point32 point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

static ZoneSet createFullZoneSet()
{
  ZoneSet zone_set;
  zone_set.header.frame_id = "test_frame_id";
  zone_set.speed_lower = 0.1;
  zone_set.speed_upper = 0.2;

  const auto p1 = createPoint32(0.0, 0.1, 0.3);
  const auto p2 = createPoint32(0.1, -0.1, 0.0);
  const auto p3 = createPoint32(0.2, 0.0, 0.3);
  const auto p4 = createPoint32(11.7, 0.2, -0.3);

  zone_set.safety1.points = { p1, p2 };
  zone_set.safety2.points = { p1, p2, p3 };
  zone_set.safety3.points = { p2, p1 };
  zone_set.warn1.points = { p1, p3, p2, p4 };
  zone_set.warn2.points = { p1, p2 };
  zone_set.muting1.points = { p3, p1 };
  zone_set.muting2.points = { p4, p2, p3 };
  return zone_set;
}

static bool markerNamespaceContains(const visualization_msgs::Marker& marker, const std::string& str)
{
  return marker.ns.find(str) != std::string::npos;
}

static bool isPolygonTypeMarker(const visualization_msgs::Marker& marker, const std::string& polygon_type)
{
  return markerNamespaceContains(marker, polygon_type);
}

static std::vector<visualization_msgs::Marker>::const_iterator
findPolygonMarker(const std::vector<visualization_msgs::Marker>& markers, const std::string& polygon_name)
{
  return std::find_if(markers.begin(), markers.end(), [&polygon_name](const auto& marker) {
    return markerNamespaceContains(marker, polygon_name);
  });
}

//////////////////
//  Test Cases  //
//////////////////

TEST(ZonesetToMarkerConversionTest, shouldReturnEmptyVectorIfNoPolygonSet)
{
  EXPECT_TRUE(toMarkers(ZoneSet()).empty());
}

TEST(ZonesetToMarkerConversionTest, shouldReturnVectorWithCorrectSizeForPartiallyFilledZoneSet)
{
  auto zone_set = createFullZoneSet();
  zone_set.muting1.points.clear();
  EXPECT_EQ(6u, toMarkers(zone_set).size());
}

TEST(ZonesetToMarkerConversionTest, shouldReturnVectorWithCorrectSizeForFullZoneSet)
{
  const auto zone_set = createFullZoneSet();
  EXPECT_EQ(7u, toMarkers(zone_set).size());
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectFrameId)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);
  for (const auto& marker : markers)
  {
    EXPECT_EQ(zone_set.header.frame_id, marker.header.frame_id);
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectNamespace)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& polygon_name : POLYGON_NAMES)
  {
    EXPECT_THAT(markers,
                ContainsMarkerWithNamespace(fmt::format(
                    "active zoneset {} min:{:+} max:{:+}", polygon_name, zone_set.speed_lower, zone_set.speed_upper)));
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectId)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    EXPECT_EQ(0, marker.id);
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectType)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    EXPECT_EQ(visualization_msgs::Marker::TRIANGLE_LIST, marker.type);
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectAction)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    EXPECT_EQ(visualization_msgs::Marker::ADD, marker.action);
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectPose)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.x);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.y);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.z);
    EXPECT_DOUBLE_EQ(1.0, marker.pose.orientation.w);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.position.x);
    EXPECT_DOUBLE_EQ(0.0, marker.pose.position.y);

    if (isPolygonTypeMarker(marker, "safety"))
    {
      EXPECT_DOUBLE_EQ(0.0, marker.pose.position.z);
    }
    else if (isPolygonTypeMarker(marker, "warn"))
    {
      EXPECT_DOUBLE_EQ(0.01, marker.pose.position.z);
    }
    else  // polygon type muting
    {
      EXPECT_DOUBLE_EQ(0.02, marker.pose.position.z);
    }
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectScale)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    EXPECT_DOUBLE_EQ(1.0, marker.scale.x);
    EXPECT_DOUBLE_EQ(1.0, marker.scale.y);
    EXPECT_DOUBLE_EQ(1.0, marker.scale.z);
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectColor)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    EXPECT_FLOAT_EQ(0.0, marker.color.r);
    EXPECT_FLOAT_EQ(0.0, marker.color.g);
    EXPECT_FLOAT_EQ(0.0, marker.color.b);
    EXPECT_FLOAT_EQ(0.4, marker.color.a);
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectPoints)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);
  const auto& name_to_polygon = generateNameToPolygonMap(zone_set);

  for (const auto& name_polygon_pair : name_to_polygon)
  {
    const auto it = findPolygonMarker(markers, name_polygon_pair.first);
    EXPECT_THAT(it->points, IsTriangleListOfArcSegment(name_polygon_pair.second.points));
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturMarkersWithAllTriangleColorsSet)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    ASSERT_THAT(marker, HasAtLeastOneTriangle());
    EXPECT_THAT(marker.colors, ::testing::SizeIs(marker.points.size() / 3));
  }
}

TEST(ZonesetToMarkerConversionTest, shouldReturnMarkersWithCorrectTriangleColors)
{
  const auto zone_set = createFullZoneSet();
  const auto markers = toMarkers(zone_set);

  for (const auto& marker : markers)
  {
    if (isPolygonTypeMarker(marker, "safety"))
    {
      EXPECT_THAT(marker.colors, AllElementsMatchColorRGBA(1, 0, 0, 1));
    }
    else if (isPolygonTypeMarker(marker, "warn"))
    {
      EXPECT_THAT(marker.colors, AllElementsMatchColorRGBA(1, 1, 0, 1));
    }
    else  // polygon type muting
    {
      EXPECT_THAT(marker.colors, AllElementsMatchColorRGBA(0, 0, 1, 1));
    }
  }
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
