#include <ros/ros.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <numeric>
#include <math.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

#include "psen_scan_v2/angle_conversions.h"

static std::string filepath;

int16_t toTenthDegree(const double& rad)
{
  return (rad / (2.0 * M_PI)) * 360 * 10;
}

class Dist
{
public:
  Dist(){};
  void update(const double& range)
  {
    data_.push_back(range);
  };

  double mean() const
  {
    double sum = std::accumulate(data_.begin(), data_.end(), 0.0);
    double mean = sum / data_.size();

    return mean;
  }

  double stdev() const
  {
    double sq_sum = std::inner_product(data_.begin(), data_.end(), data_.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / data_.size() - mean() * mean());

    return stdev;
  }

  double pdf(const double& v) const
  {
    return (1.0 / (mean() * sqrt(2 * M_PI))) * exp(-0.5 * pow((v - mean()) / stdev(), 2));
  }

  double last() const
  {
    return data_.back();
  }

private:
  std::vector<double> data_;
};

class Comparator
{
public:
  Comparator(ros::NodeHandle& nh, std::map<int16_t, Dist>& dists) : nh_(nh), dists_(dists)
  {
    sub_ = nh_.subscribe("/laser_scanner/scan", 1000, &Comparator::scanCb, this);
  };

  void scanCb(const sensor_msgs::LaserScanConstPtr& scan)
  {
    // TODO compare scan!

    counter_++;
  }

  void run()
  {
    ros::Rate r(10);
    while (ros::ok() && counter_ < 100)
    {
      r.sleep();
    }

    std::cerr << counter_ << "Scans received. Run stoped\n";
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::map<int16_t, Dist> dists_;
  size_t counter_{ 0 };
};

std::map<int16_t, Dist> loadBin(std::string filepath)
{
  std::map<int16_t, Dist> bins;

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
      std::cout << scan->angle_min << " " << scan->angle_max << " " << (int)toTenthDegree(scan->angle_min) << " "
                << (int)toTenthDegree(scan->angle_max) << std::endl;

      for (size_t i = 0; i < scan->ranges.size(); ++i)
      {
        auto bin_addr = toTenthDegree(scan->angle_min + scan->angle_increment * i);

        if (bins.find(bin_addr) == bins.end())
        {
          bins.emplace(bin_addr, Dist{});
          // Create bin
        }
        bins[bin_addr].update(scan->ranges[i]);
      }
    }
  }

  bag.close();

  return bins;
}

TEST(CompareTest, simpleCompare)
{
  ros::NodeHandle nh;

  auto bins = loadBin(filepath);
  Comparator cp(nh, bins);
  cp.run();

  std::cerr << "Test done!\n";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "scan_compare_test");
  ros::NodeHandle nh("~");

  std::string testfile_path;
  nh.getParam("testfile", testfile_path);

  ROS_ERROR_STREAM("Using testfile " << testfile_path);

  if (!boost::filesystem::exists(testfile_path))
  {
    ROS_ERROR_STREAM("File " << testfile_path << " not found!");
    return -1;
  }

  filepath = testfile_path;

  auto bins = loadBin(filepath);

  for (auto const& bin : bins)
  {
    std::cout << (int)bin.first << " mean: " << bin.second.mean() << " std: " << bin.second.stdev()
              << "  last: " << bin.second.last() << "  pdf:" << bin.second.pdf(bin.second.last()) << "\n";
  }

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  return RUN_ALL_TESTS();
}