cmake_minimum_required(VERSION 3.0.2)
project(test_psen_scan_v2_prerelease)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(psen_scan_v2 REQUIRED)

  ####################
  ##  Startup-Test  ##
  ####################
  # run the default launch file to prevent segfaults or missing libraries
  add_rostest(test/integrationtest_ros_scanner_launch.test)

endif()
