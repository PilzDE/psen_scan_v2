#include <ros/ros.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>

#include <future>
#include <iostream>

#include <numeric>
#include <math.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/dist.h"
#include "psen_scan_v2/message_collector.h"

namespace psen_scan_v2_test
{
int16_t toTenthDegree(const double& rad)
{
  return (rad / (2.0 * M_PI)) * 360 * 10;
}

std::map<int16_t, NormalDist> binsFromScans(std::vector<sensor_msgs::LaserScanConstPtr> scans)
{
  std::map<int16_t, NormalDist> bins;

  for (const auto& scan : scans)
  {
    if (scan == nullptr)
    {
      continue;
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i)
    {
      auto bin_addr = toTenthDegree(scan->angle_min + scan->angle_increment * i);

      if (bins.find(bin_addr) == bins.end())
      {
        bins.emplace(bin_addr, NormalDist{});
        // Create bin
      }
      bins[bin_addr].update(scan->ranges[i]);
    }
  }

  return bins;
}

std::map<int16_t, NormalDist> binsFromRosbag(std::string filepath)
{
  std::map<int16_t, NormalDist> bins;

  rosbag::Bag bag;
  bag.open(filepath, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/laser_scanner/scan"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (rosbag::MessageInstance const m : view)
  {
    sensor_msgs::LaserScanConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
    if (scan != nullptr)
    {
      for (size_t i = 0; i < scan->ranges.size(); ++i)
      {
        auto bin_addr = toTenthDegree(scan->angle_min + scan->angle_increment * i);

        if (bins.find(bin_addr) == bins.end())
        {
          bins.emplace(bin_addr, NormalDist{});
          // Create bin
        }
        bins[bin_addr].update(scan->ranges[i]);
      }
    }
  }

  bag.close();

  return bins;
}

class ScanComparisionTests : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    ros::NodeHandle pnh{ "~" };

    std::string filepath;
    pnh.getParam("testfile", filepath);

    ROS_INFO_STREAM("Using testfile " << filepath);
    if (!boost::filesystem::exists(filepath))
    {
      ROS_ERROR_STREAM("File " << filepath << " not found!");
      FAIL();
    }

    bins_set_expected_ = binsFromRosbag(filepath);
  }

  static std::map<int16_t, NormalDist> bins_set_expected_;
};

std::map<int16_t, NormalDist> ScanComparisionTests::bins_set_expected_{};

TEST_F(ScanComparisionTests, simpleCompare)
{
  ros::NodeHandle nh;

  size_t sample_size = 200;

  auto scans = MessageCollector<sensor_msgs::LaserScan>(nh).collectScans(sample_size, "/laser_scanner/scan");
  auto bins_actual = binsFromScans(scans);

  for (const auto& bin : bins_actual)
  {
    if (bins_set_expected_.find(bin.first) == bins_set_expected_.end())
    {
      FAIL() << "Did not find expected value for angle " << bin.first / 10. << " in the given reference scan\n";
    }

    auto distance = bhattacharyya_distance(bin.second, bins_set_expected_.at(bin.first));
    EXPECT_LE(distance, 2.) << " on angle " << bin.first / 10.0 << " deg the measured value " << bin.second
                            << "deviates from value obtained from the reference " << bins_set_expected_.at(bin.first);
  }
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "scan_compare_test");

  // Needed since we use a subscriber
  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  return RUN_ALL_TESTS();
}