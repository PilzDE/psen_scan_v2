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
  Comparator(ros::NodeHandle& nh) : nh_(nh){};

  void scanCb(const sensor_msgs::LaserScanConstPtr& scan)
  {
    scans_.push_back(scan);
    counter_++;
  }

  void collectScans(size_t sample_size)
  {
    sub_ = nh_.subscribe("/laser_scanner/scan", 1000, &Comparator::scanCb, this);

    ros::Rate r(10);
    while (ros::ok() && counter_ < sample_size)
    {
      r.sleep();
    }

    std::cerr << counter_ << "Scans received. Run stoped\n";
  }

  std::map<int16_t, Dist> getBins()
  {
    std::map<int16_t, Dist> bins;

    for (const auto& scan : scans_)
    {
      if (scan == nullptr)
      {
        continue;
      }

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

    return bins;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::vector<sensor_msgs::LaserScanConstPtr> scans_;
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

  auto bins_expected = loadBin(filepath);

  size_t sample_size = 100;

  Comparator cp(nh);
  cp.collectScans(sample_size);  // This is not exact, ok for now...
  auto bins_actual = cp.getBins();

  for (const auto& bin : bins_actual)
  {
    if (bins_expected.find(bin.first) == bins_expected.end())
    {
      std::cerr << "Did not find expected value for " << bin.first / 10. << "\n";
      continue;
    }

    const auto bin_expect = bins_expected.at(bin.first);

    std::cerr << "Comparing degree [" << bin.first / 10. << "] \n"
              << "actual: mean: " << bin.second.mean() << " stdev:" << bin.second.stdev() << "\n"
              << "expect: mean: " << bin_expect.mean() << " stdev:" << bin_expect.stdev() << "\n\n";
  }

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