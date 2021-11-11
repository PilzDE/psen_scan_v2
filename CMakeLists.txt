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

cmake_minimum_required(VERSION 3.7)
project(psen_scan_v2)

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  rosconsole_bridge
  roscpp
  sensor_msgs
  cmake_modules # Needed for find_package(TinyXML2 REQUIRED)
  message_generation
  geometry_msgs
  visualization_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost 1.69) ## Boost::system is header-only since 1.69
if(NOT Boost_FOUND)
  #catkin_lint: ignore_once shadowed_find
  find_package(Boost REQUIRED COMPONENTS system)
endif()
find_package(console_bridge REQUIRED)
find_package(fmt REQUIRED)

find_package(TinyXML2 REQUIRED)

add_message_files(
  FILES
  ZoneSet.msg
  ZoneSetConfiguration.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  visualization_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    message_runtime
    geometry_msgs
    std_msgs
    visualization_msgs
  DEPENDS TinyXML2
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

######################
## Build Standalone ##
######################

include_directories(
  include
  standalone/include
  ${console_bridge_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

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

add_library(
  ${PROJECT_NAME}_standalone
  ${${PROJECT_NAME}_standalone_sources}
)

target_link_libraries(${PROJECT_NAME}_standalone
  ${Boost_LIBRARIES}
  ${console_bridge_LIBRARIES}
  fmt::fmt
)
target_include_directories(${PROJECT_NAME}_standalone PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_library(
  ${PROJECT_NAME}_standalone_xml_configuration_import
  standalone/src/configuration/xml_configuration_parsing.cpp
)

target_link_libraries(${PROJECT_NAME}_standalone_xml_configuration_import
  ${console_bridge_LIBRARIES}
  ${TinyXML_LIBRARIES}
  ${TinyXML2_LIBRARIES}
  fmt::fmt
)
target_include_directories(${PROJECT_NAME}_standalone_xml_configuration_import PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

###########
## Build ##
###########

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_sources})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_standalone
)


add_executable(config_server_node
  src/config_server_node_main.cpp
  src/config_server_node.cpp)
target_link_libraries(config_server_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_standalone_xml_configuration_import
)
add_dependencies(config_server_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)


add_executable(active_zoneset_node
  src/active_zoneset_node_main.cpp
  src/active_zoneset_node.cpp
)
target_link_libraries(active_zoneset_node
  ${catkin_LIBRARIES}
  fmt::fmt
)
add_dependencies(active_zoneset_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
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
  ${PROJECT_NAME}_standalone
  config_server_node
  active_zoneset_node
  ${PROJECT_NAME}_standalone_xml_configuration_import
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/zonesets_visualization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

############
##  Test  ##
############
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(rosbag REQUIRED)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    append_coverage_compiler_flags()
  endif()

  roslaunch_add_file_check(launch DEPENDENCIES ${PROJECT_NAME}_node)

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
    fmt::fmt
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
    fmt::fmt
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
    fmt::fmt
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

  catkin_add_gtest(unittest_monitoring_frame_msg_stamped
    standalone/test/unit_tests/data_conversion_layer/unittest_monitoring_frame_msg_stamped.cpp
    standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
    standalone/src/data_conversion_layer/diagnostics.cpp
  )
  target_link_libraries(unittest_monitoring_frame_msg_stamped
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

  catkin_add_gmock(unittest_xml_configuration_parser
    standalone/test/unit_tests/configuration/unittest_xml_configuration_parser.cpp
  )
  target_link_libraries(unittest_xml_configuration_parser
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_standalone_xml_configuration_import
    ${console_bridge_LIBRARIES}
    fmt::fmt
  )
  add_dependencies(unittest_xml_configuration_parser ${${PROJECT_NAME}_EXPORTED_TARGETS})

  file(COPY standalone/test/unit_tests/configuration/unittest_xml_configuration_parser-testfile-no-speedrange.xml
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/testfiles)
  file(COPY standalone/test/unit_tests/configuration/unittest_xml_configuration_parser-testfile-with-speedrange.xml
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/testfiles)


  catkin_add_gmock(unittest_zone_configuration_ros_conversion
    test/unit_tests/unittest_zone_configuration_ros_conversion.cpp
  )
  target_link_libraries(unittest_zone_configuration_ros_conversion
    ${catkin_LIBRARIES}
    fmt::fmt
  )
  add_dependencies(unittest_zone_configuration_ros_conversion ${${PROJECT_NAME}_EXPORTED_TARGETS})

  catkin_add_gmock(unittest_zoneset_to_marker_conversion
    test/unit_tests/unittest_zoneset_to_marker_conversion.cpp
  )
  target_link_libraries(unittest_zoneset_to_marker_conversion
    ${catkin_LIBRARIES}
    fmt::fmt
  )
  add_dependencies(unittest_zoneset_to_marker_conversion
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
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
    fmt::fmt
  )

  add_rostest_gmock(integrationtest_config_server_node
    test/integration_tests/integrationtest_config_server_node.test
    test/integration_tests/integrationtest_config_server_node.cpp
    src/config_server_node.cpp
  )
  target_link_libraries(integrationtest_config_server_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_standalone_xml_configuration_import
    ${rosbag_LIBRARIES}
  )
  add_dependencies(integrationtest_config_server_node
    ${PROJECT_NAME}_standalone
    config_server_node
    ${PROJECT_NAME}_standalone_xml_configuration_import
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

  add_rostest_gmock(integrationtest_active_zoneset_node
    test/integration_tests/integrationtest_active_zoneset_node.test
    test/integration_tests/integrationtest_active_zoneset_node.cpp
  )
  target_link_libraries(integrationtest_active_zoneset_node
    ${catkin_LIBRARIES}
    ${rosbag_LIBRARIES}
    fmt::fmt
  )
  add_dependencies(integrationtest_active_zoneset_node
    ${PROJECT_NAME}_standalone
    active_zoneset_node
    ${PROJECT_NAME}_standalone_xml_configuration_import
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )

  add_rostest_gmock(unittest_ros_parameter_handler
      test/unit_tests/unittest_ros_parameter_handler.test
      test/unit_tests/unittest_ros_parameter_handler.cpp
  )
  target_link_libraries(unittest_ros_parameter_handler
    ${catkin_LIBRARIES}
  )

  ######################
  ##  Hardware-Tests  ##
  ######################

  # See ./hwtest_readme.md for details on the hardware tests.
  if(ENABLE_HARDWARE_TESTING)

    add_rostest(test/hw_tests/hwtest_publish.test)

    add_rostest(test/hw_tests/hwtest_scan_range.test
      ARGS config:=default.yaml)
    add_rostest(test/hw_tests/hwtest_scan_range.test
      ARGS config:=subrange.yaml)
    add_rostest(test/hw_tests/hwtest_scan_range.test
      ARGS config:=with_intensities.yaml)
    add_rostest(test/hw_tests/hwtest_scan_range.test
      ARGS config:=big_resolution.yaml)
  endif()

  ###########################################
  ##  Hardware-Tests with saintsmart_relay ##
  ###########################################

  if(ENABLE_HARDWARE_TESTING_WITH_RELAY)

    add_rostest_gmock(hwtest_zoneset_switching
      test/hw_tests/hwtest_zoneset_switching.test
      test/hw_tests/hwtest_zoneset_switching.cpp
    )

  else()

    catkin_add_executable_with_gmock(hwtest_zoneset_switching
                                    test/hw_tests/hwtest_zoneset_switching.cpp
                                    EXCLUDE_FROM_ALL)
    add_dependencies(tests hwtest_zoneset_switching)

  endif()


  target_link_libraries(hwtest_zoneset_switching
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_standalone
    ${rosbag_LIBRARIES}
    fmt::fmt
  )

  #########################################
  ##  Hardware-Tests in Test Environment ##
  #########################################

  #---------------------------------
  ### hwtest_scan_compare_standalone
  #---------------------------------

  # This test is intended to be run on in a dedicated environment with a prerecorded reference scan
  # and thus needs to be enabled seperately.
  if(ENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN)

    catkin_add_gtest(hwtest_scan_compare_standalone
      test/hw_tests/hwtest_scan_compare_standalone.cpp
      standalone/src/laserscan.cpp
    )

  else()
    # always at least build the test to avoid build breaking changes
    catkin_add_executable_with_gtest(hwtest_scan_compare_standalone
                                            test/hw_tests/hwtest_scan_compare_standalone.cpp
                                            standalone/src/laserscan.cpp
                                            EXCLUDE_FROM_ALL)
    add_dependencies(tests hwtest_scan_compare_standalone)

  endif()

  target_link_libraries(hwtest_scan_compare_standalone
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_standalone
    ${rosbag_LIBRARIES}
    fmt::fmt
  )


  #----------------------
  ### hwtest_scan_compare
  #----------------------

  if(ENABLE_HARDWARE_TESTING_WITH_REFERENCE_SCAN)

    add_rostest_gtest(hwtest_scan_compare
      test/hw_tests/hwtest_scan_compare.test
      test/hw_tests/hwtest_scan_compare.cpp
      standalone/src/laserscan.cpp
    )

  else()
    # always at least build the test to avoid build breaking changes
    catkin_add_executable_with_gtest(hwtest_scan_compare
    test/hw_tests/hwtest_scan_compare.cpp
    standalone/src/laserscan.cpp
    EXCLUDE_FROM_ALL
    )
    add_dependencies(tests hwtest_scan_compare)
  endif()


  target_link_libraries(hwtest_scan_compare
    ${catkin_LIBRARIES}
    ${rosbag_LIBRARIES}
    fmt::fmt
  )


  #------------------------------
  ### hwtest_timestamp_standalone
  #------------------------------

  # This test is intended to be run with a parallel wireshark process and thus needs to be enabled separately.
  if(ENABLE_HARDWARE_TESTING_WITH_WIRESHARK)
    catkin_add_gtest(hwtest_timestamp_standalone
      test/hw_tests/hwtest_timestamp_standalone.cpp
      test/src/test_data.cpp
    )
  else()
    # always at least build the test to avoid build breaking changes
    catkin_add_executable_with_gtest(hwtest_timestamp_standalone
      test/hw_tests/hwtest_timestamp_standalone.cpp
      test/src/test_data.cpp
      EXCLUDE_FROM_ALL
    )
    add_dependencies(tests hwtest_timestamp_standalone)
  endif()

  target_link_libraries(hwtest_timestamp_standalone
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}_standalone
  )


  # generate coverage report using CATKIN_MAKE:
  # catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug psen_scan_v2_coverage (adding -j1 recommended)
  #
  # generate coverage report using CATKIN:
  # catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
  # catkin build
  # catkin build psen_scan_v2 -j1 -v --no-deps --catkin-make-args psen_scan_v2_coverage
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/standalone/test*"
                          "*/${PROJECT_NAME}/test*"
                          "*/${PROJECT_NAME}/src/psen_scan_driver.cpp"
                          "*/ZoneSet.h" # generated message
                          "*/ZoneSetConfiguration.h" # generated message
        )
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()
