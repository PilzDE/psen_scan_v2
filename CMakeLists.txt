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

cmake_minimum_required(VERSION 3.0.2)
project(psen_scan_v2)

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

find_package(catkin REQUIRED COMPONENTS
  rosconsole_bridge
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
)


###############
## Sanitizer ##
###############
# to run (only one sanitizer at a time):
# 1. Set environment variables for log_path in your shell
# export ASAN_OPTIONS="log_path=/path/to/asan-logs"
# export TSAN_OPTIONS="log_path=/path/to/tsan-logs"
# export UBSAN_OPTIONS="log_path=/path/to/ubsan-logs"
# 2. Compile
# catkin_make -DCATKIN_SANITIZER=address|thread|undefined
# 3. Run the node
# 4. Check log_path folder for contents
# build and devel folders have to be deleted before run
if(CATKIN_SANITIZER)
  set(SANITIZER_OPTIONS "address|thread|undefined")
  if("${CATKIN_SANITIZER}" MATCHES "${SANITIZER_OPTIONS}")
    message(STATUS "Activated: ${CATKIN_SANITIZER}-sanitizer")
    add_compile_options(-fsanitize=${CATKIN_SANITIZER} -g)
  else()
    message(FATAL_ERROR "${CATKIN_SANITIZER} is not a valid sanitizer. Valid options are: [${SANITIZER_OPTIONS}]")
  endif()
endif()

################
## Clang tidy ##
################
# to run: catkin_make -DCATKIN_ENABLE_CLANG_TIDY=true
# build and devel folders have to be deleted before run
if(CATKIN_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
  endif()
endif()

###########
## Build ##
###########

#catkin_lint: ignore duplicate_cmd
#catkin_lint: ignore subproject
add_subdirectory(standalone)
#catkin_lint: report duplicate_cmd
#catkin_lint: report subproject

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_sources})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_standalone
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/ standalone/include/${PROJECT_NAME}_standalone/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

install(TARGETS
  ${PROJECT_NAME}_node
  # the following requires cmake 3.13
  #${PROJECT_NAME}_standalone
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

############
##  Test  ##
############
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(pilz_testutils REQUIRED)
  find_package(fmt REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    append_coverage_compiler_flags()
  endif()

  roslaunch_add_file_check(launch DEPENDENCIES ${PROJECT_NAME}_node)

  include_directories(SYSTEM ${pilz_testutils_INCLUDE_DIRS})
  include_directories(
    standalone/include
    standalone/test/include
    test/include
    ${catkin_INCLUDE_DIR}
  )

  ##################
  ##  Unit-Tests  ##
  ##################
  catkin_add_gtest(unittest_new_exceptions
    test/unit_tests/unittest_new_exceptions.cpp
  )
  target_link_libraries(unittest_new_exceptions
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_raw_processing
    standalone/test/unit_tests/data_conversion_layer/unittest_raw_processing.cpp
  )
  target_link_libraries(unittest_raw_processing
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gtest(unittest_start_request
    standalone/test/unit_tests/data_conversion_layer/unittest_start_request.cpp
    standalone/src/data_conversion_layer/start_request.cpp
    standalone/src/data_conversion_layer/start_request_serialization.cpp
  )
  target_link_libraries(unittest_start_request
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_stop_request
    standalone/test/unit_tests/data_conversion_layer/unittest_stop_request.cpp
    standalone/src/data_conversion_layer/stop_request_serialization.cpp
  )
  target_link_libraries(unittest_stop_request
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_scan_range
    standalone/test/unit_tests/util/unittest_scan_range.cpp
  )
  target_link_libraries(unittest_scan_range
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_scanner_configuration
    standalone/test/unit_tests/configuration/unittest_scanner_configuration.cpp
  )
  target_link_libraries(unittest_scanner_configuration
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_tenth_of_degree
    standalone/test/unit_tests/util/unittest_tenth_of_degree.cpp
  )

  catkin_add_gmock(unittest_udp_client
    standalone/test/unit_tests/communication_layer/unittest_udp_client.cpp
  )
  target_link_libraries(unittest_udp_client
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gtest(unittest_tenth_degree_conversion
    standalone/test/unit_tests/data_conversion_layer/unittest_tenth_degree_conversion.cpp
  )
  target_link_libraries(unittest_tenth_degree_conversion
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_laserscan
    standalone/test/unit_tests/api/unittest_laserscan.cpp
    standalone/src/laserscan.cpp
  )
  target_link_libraries(unittest_laserscan
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_laserscan_conversions
    standalone/test/unit_tests/data_conversion_layer/unittest_laserscan_conversions.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_link_libraries(unittest_laserscan_conversions
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gtest(unittest_laserscan_ros_conversions
    test/unit_tests/unittest_laserscan_ros_conversions.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_link_libraries(unittest_laserscan_ros_conversions
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gtest(unittest_scanner_reply_msg
    standalone/test/unit_tests/data_conversion_layer/unittest_scanner_reply_msg.cpp
    standalone/src/data_conversion_layer/scanner_reply_serialization_deserialization.cpp
  )
  target_link_libraries(unittest_scanner_reply_msg
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gtest(unittest_monitoring_frame_msg
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
  )
  target_link_libraries(unittest_monitoring_frame_msg
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gtest(unittest_monitoring_frame_serialization_deserialization
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_serialization_deserialization.cpp
    standalone/test/src/data_conversion_layer/monitoring_frame_serialization.cpp
  )
  target_link_libraries(unittest_monitoring_frame_serialization_deserialization
    ${catkin_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gmock(unittest_logging
    standalone/test/unit_tests/util/unittest_logging.cpp
  )
  target_link_libraries(unittest_logging
    ${catkin_LIBRARIES}
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gmock(unittest_monitoring_frame_diagnostic_message
    standalone/src/data_conversion_layer/diagnostics.cpp
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_diagnostic_message.cpp
  )
  target_link_libraries(unittest_monitoring_frame_diagnostic_message
    ${catkin_LIBRARIES}
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )

  #########################
  ##  Integration-Tests  ##
  #########################
  catkin_add_gmock(integrationtest_udp_client
    standalone/test/integration_tests/communication_layer/integrationtest_udp_client.cpp
    standalone/test/src/communication_layer/mock_udp_server.cpp
  )
  target_link_libraries(integrationtest_udp_client
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
    fmt::fmt
  )

  catkin_add_gmock(integrationtest_scanner_api
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
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
    fmt::fmt
  )

  add_rostest_gmock(integrationtest_ros_scanner_node
    test/integration_tests/integrationtest_ros_scanner_node.test
    test/integration_tests/integrationtest_ros_scanner_node.cpp
    standalone/src/laserscan.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_link_libraries(integrationtest_ros_scanner_node
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
    fmt::fmt
  )

  add_rostest_gmock(unittest_ros_parameter_handler
      test/unit_tests/unittest_ros_parameter_handler.test
      test/unit_tests/unittest_ros_parameter_handler.cpp
  )
  target_link_libraries(unittest_ros_parameter_handler
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
  )

  ######################
  ##  Hardware-Tests  ##
  ######################

  # build this using catkin_make -DENABLE_HARDWARE_TESTING=ON
  if(ENABLE_HARDWARE_TESTING)
    find_package(rosbag REQUIRED)

    add_rostest_gtest(hwtest_scan_compare
        test/hw_tests/hwtest_scan_compare.test
        test/hw_tests/hwtest_scan_compare.cpp
        standalone/src/laserscan.cpp
    )
    target_link_libraries(hwtest_scan_compare
      ${catkin_LIBRARIES}
      ${pilz_testutils_LIBRARIES}
      ${rosbag_LIBRARIES}
      fmt::fmt
    )
  endif()


  # generate coverage report using CATKIN_MAKE:
  # catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug psen_scan_v2_coverage (adding -j1 recommended)
  #
  # generate coverage report using CATKIN:
  # catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
  # catkin build
  # catkin build psen_scan_v2 -v --no-deps --catkin-make-args psen_scan_v2_coverage
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/standalone/test*"
                          "*/${PROJECT_NAME}/test*"
                          "*/${PROJECT_NAME}/src/psen_scan_driver.cpp")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()
