# Copyright (c) 2020-2021 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.5)
project(psen_scan_v2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost 1.69)
if(NOT Boost_FOUND)
  find_package(Boost REQUIRED COMPONENTS system filesystem)
endif()
find_package(fmt REQUIRED)

################
## Clang tidy ##
################
# alternative to ament_cmake_clang_tidy which is very time consuming
# to run: catkin_make -DAMENT_ENABLE_CLANG_TIDY=true
# build and install folders have to be deleted before run
if(AMENT_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}") # Todo: -header-filter=${PROJECT_NAME}/.*
  endif()
endif()

######################
## Build Standalone ##
######################

set(${PROJECT_NAME}_standalone_sources
  standalone/src/scanner_v2.cpp
  standalone/src/laserscan.cpp
  standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
  standalone/src/data_conversion_layer/start_request.cpp
  standalone/src/data_conversion_layer/start_request_serialization.cpp
  standalone/src/data_conversion_layer/stop_request_serialization.cpp
  standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
  standalone/src/data_conversion_layer/diagnostics.cpp
  standalone/src/data_conversion_layer/scanner_reply_serialization_deserialization.cpp
)

add_library(${PROJECT_NAME}_standalone
  ${${PROJECT_NAME}_standalone_sources}
)
target_compile_definitions(${PROJECT_NAME}_standalone
  PUBLIC
  _ROS_BUILD_
)
target_include_directories(${PROJECT_NAME}_standalone
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/standalone/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_standalone
  Boost
  fmt
  rcutils
)

###########
## Build ##
###########

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_node_sources}
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_standalone
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
)

#############
## Install ##
#############

install(DIRECTORY include/ standalone/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_standalone
  DESTINATION lib/${PROJECT_NAME}
)

############
##  Test  ##
############
if(BUILD_TESTING)

  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(console_bridge REQUIRED)
  find_package(ros_testing REQUIRED)

  include_directories(
    include
    standalone/include
    standalone/test/include
    test/include
  )

  ####################
  ##  Linter-Tests  ##
  ####################
  ament_clang_format(CONFIG_FILE ".clang-format")
  ament_xmllint()

  ##################
  ##  Unit-Tests  ##
  ##################
  ament_add_gtest(unittest_new_exceptions
    test/unit_tests/unittest_new_exceptions.cpp
  )

  ament_add_gtest(unittest_raw_processing
    standalone/test/unit_tests/data_conversion_layer/unittest_raw_processing.cpp
  )
  target_link_libraries(unittest_raw_processing
    fmt::fmt
  )

  ament_add_gtest(unittest_start_request
    standalone/test/unit_tests/data_conversion_layer/unittest_start_request.cpp
    standalone/src/data_conversion_layer/start_request.cpp
    standalone/src/data_conversion_layer/start_request_serialization.cpp
  )
  target_link_libraries(unittest_start_request
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_stop_request
    standalone/test/unit_tests/data_conversion_layer/unittest_stop_request.cpp
    standalone/src/data_conversion_layer/stop_request_serialization.cpp
  )

  ament_add_gtest(unittest_scan_range
    standalone/test/unit_tests/util/unittest_scan_range.cpp
  )

  ament_add_gtest(unittest_scanner_configuration
    standalone/test/unit_tests/configuration/unittest_scanner_configuration.cpp
  )
  target_link_libraries(unittest_scanner_configuration
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_tenth_of_degree
    standalone/test/unit_tests/util/unittest_tenth_of_degree.cpp
  )

  ament_add_gmock(unittest_udp_client
    standalone/test/unit_tests/communication_layer/unittest_udp_client.cpp
  )
  target_link_libraries(unittest_udp_client
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_tenth_degree_conversion
    standalone/test/unit_tests/data_conversion_layer/unittest_tenth_degree_conversion.cpp
  )

  ament_add_gtest(unittest_laserscan
    standalone/test/unit_tests/api/unittest_laserscan.cpp
    standalone/src/laserscan.cpp
  )
  target_link_libraries(unittest_laserscan
    fmt::fmt
  )

  ament_add_gtest(unittest_laserscan_conversions
    standalone/test/unit_tests/data_conversion_layer/unittest_laserscan_conversions.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_link_libraries(unittest_laserscan_conversions
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_laserscan_ros_conversions
    test/unit_tests/unittest_laserscan_ros_conversions.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  ament_target_dependencies(unittest_laserscan_ros_conversions
    rclcpp
    sensor_msgs
  )
  target_link_libraries(unittest_laserscan_ros_conversions
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_scanner_reply_msg
    standalone/test/unit_tests/data_conversion_layer/unittest_scanner_reply_msg.cpp
    standalone/src/data_conversion_layer/scanner_reply_serialization_deserialization.cpp
  )
  target_link_libraries(unittest_scanner_reply_msg
    fmt::fmt
  )

  ament_add_gtest(unittest_monitoring_frame_msg
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
  )
  target_link_libraries(unittest_monitoring_frame_msg
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_monitoring_frame_msg_stamped
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_msg_stamped.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_link_libraries(unittest_monitoring_frame_msg_stamped
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gtest(unittest_monitoring_frame_serialization_deserialization
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_serialization_deserialization.cpp
    standalone/test/src/data_conversion_layer/monitoring_frame_serialization.cpp
  )
  target_link_libraries(unittest_monitoring_frame_serialization_deserialization
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gmock(unittest_logging
    standalone/test/unit_tests/util/unittest_logging.cpp
  )
  target_link_libraries(unittest_logging
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gmock(unittest_monitoring_frame_diagnostic_message
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_diagnostic_message.cpp
  )
  target_link_libraries(unittest_monitoring_frame_diagnostic_message
    fmt::fmt
  )

  #########################
  ##  Integration-Tests  ##
  #########################
  ament_add_gmock(integrationtest_udp_client
    standalone/test/integration_tests/communication_layer/integrationtest_udp_client.cpp
    standalone/test/src/communication_layer/mock_udp_server.cpp
  )
  target_link_libraries(integrationtest_udp_client
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_gmock(integrationtest_scanner_api
    standalone/test/integration_tests/api/integrationtest_scanner_api.cpp
    standalone/test/src/communication_layer/mock_udp_server.cpp
    standalone/test/src/communication_layer/scanner_mock.cpp
    standalone/test/src/data_conversion_layer/monitoring_frame_serialization.cpp
    standalone/src/scanner_v2.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
    standalone/src/data_conversion_layer/start_request.cpp
    standalone/src/data_conversion_layer/stop_request_serialization.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/src/data_conversion_layer/start_request_serialization.cpp
    standalone/src/data_conversion_layer/scanner_reply_serialization_deserialization.cpp
  )
  target_link_libraries(integrationtest_scanner_api
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  ament_add_ros_isolated_gmock(integrationtest_ros_scanner_node
    test/integration_tests/integrationtest_ros_scanner_node.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_compile_definitions(integrationtest_ros_scanner_node
    PUBLIC
    _ROS_BUILD_
  )
  target_link_libraries(integrationtest_ros_scanner_node
    fmt::fmt
  )
  ament_target_dependencies(integrationtest_ros_scanner_node
    rclcpp
    sensor_msgs
  )

  ament_add_ros_isolated_gmock(unittest_ros_parameter_handler
      test/unit_tests/unittest_ros_parameter_handler.cpp
  )
  ament_target_dependencies(unittest_ros_parameter_handler
    rclcpp
  )

  add_ros_test(test/integration_tests/integrationtest_ros_scanner_launch.py)

  ######################
  ##  Hardware-Tests  ##
  ######################

  # See ./hwtest_readme.md for details on the hardware tests.
  if(ENABLE_HARDWARE_TESTING)

    add_ros_test(test/hw_tests/hwtest_publish.py)

    # TODO
    # add_rostest(test/hw_tests/hwtest_scan_range.test
    #   ARGS config:=default.yaml)
    # add_rostest(test/hw_tests/hwtest_scan_range.test
    #   ARGS config:=subrange.yaml)
    # add_rostest(test/hw_tests/hwtest_scan_range.test
    #   ARGS config:=with_intensities.yaml)
    # add_rostest(test/hw_tests/hwtest_scan_range.test
    #   ARGS config:=big_resolution.yaml)
  endif()

  #########################################
  ##  Hardware-Tests in Test Environment ##
  #########################################

  #---------------------------------
  ### hwtest_scan_compare_standalone
  #---------------------------------

  # This test is intended to be run on in a dedicated environment with a prerecorded reference scan
  # and thus needs to be enabled seperately.
  find_package(rosbag2_cpp REQUIRED)
  if(ENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN)

    ament_add_gtest(hwtest_scan_compare_standalone
      test/hw_tests/hwtest_scan_compare_standalone.cpp
      standalone/src/laserscan.cpp
    )
  else()

    # always at least build the test to avoid build breaking changes
    ament_add_gtest(hwtest_scan_compare_standalone
      test/hw_tests/hwtest_scan_compare_standalone.cpp
      standalone/src/laserscan.cpp
      SKIP_TEST
    )
  endif()

  target_link_libraries(hwtest_scan_compare_standalone
    ${PROJECT_NAME}_standalone
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )
  ament_target_dependencies(hwtest_scan_compare_standalone
    rosbag2_cpp
    rclcpp
    sensor_msgs
  )

  #----------------------
  ### hwtest_scan_compare
  #----------------------

  ament_add_gtest_executable(hwtest_scan_compare
    test/hw_tests/hwtest_scan_compare.cpp
    standalone/src/laserscan.cpp
  )
  target_link_libraries(hwtest_scan_compare
    ${PROJECT_NAME}_standalone
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )
  ament_target_dependencies(hwtest_scan_compare
    rosbag2_cpp
    rclcpp
    sensor_msgs
  )

  if(ENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN)

    add_ros_test(test/hw_tests/hwtest_scan_compare.py
      ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}" "test_duration:=10"
    )
  endif()


  #------------------------------
  ### hwtest_timestamp_standalone
  #------------------------------

  # This test is intended to be run with a parallel wireshark process and thus needs to be enabled separately.
  if(ENABLE_HARDWARE_TESTING_WITH_WIRESHARK)
    ament_add_gtest(hwtest_timestamp_standalone
      test/hw_tests/hwtest_timestamp_standalone.cpp
      test/src/test_data.cpp
    )
  else()
    # always at least build the test to avoid build breaking changes
    ament_add_gtest(hwtest_timestamp_standalone
      test/hw_tests/hwtest_timestamp_standalone.cpp
      test/src/test_data.cpp
      SKIP_TEST
    )
  endif()

  target_link_libraries(hwtest_timestamp_standalone
    ${PROJECT_NAME}_standalone
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )
endif()

ament_package()
