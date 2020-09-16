# Copyright (c) 2020 Pilz GmbH & Co. KG
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

cmake_minimum_required(VERSION 2.8.12)
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

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(console_bridge REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    rosconsole_bridge
    roscpp
    sensor_msgs
  DEPENDS
    console_bridge
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_sources
  src/controller_state_machine.cpp
  src/laserscan.cpp
  src/start_request.cpp
)

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
  src/ros_parameter_handler.cpp
  src/scanner_configuration.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME}
  ${console_bridge_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_node_sources})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
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

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    append_coverage_compiler_flags()
  endif()

  roslaunch_add_file_check(launch DEPENDENCIES ${PROJECT_NAME}_node)

  include_directories(include
    test/include
    ${pilz_testutils_INCLUDE_DIRS}
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
    gtest_main
  )

  catkin_add_gmock(unittest_msg_decoder
    test/unit_tests/unittest_msg_decoder.cpp
  )
  target_link_libraries(unittest_msg_decoder
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_start_request
    test/unit_tests/unittest_start_request.cpp
    src/start_request.cpp
    src/scanner_configuration.cpp
  )
  target_link_libraries(unittest_start_request
    ${catkin_LIBRARIES}
  )

  catkin_add_gmock(unittest_controller_state_machine
    test/unit_tests/unittest_controller_state_machine.cpp
    src/controller_state_machine.cpp
  )
  target_link_libraries(unittest_controller_state_machine
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_scanner_configuration
    test/unit_tests/unittest_scanner_configuration.cpp
    src/scanner_configuration.cpp
  )
  target_link_libraries(unittest_scanner_configuration
    ${catkin_LIBRARIES}
  )

  catkin_add_gmock(unittest_scanner
    test/unit_tests/unittest_scanner.cpp
    src/scanner_configuration.cpp
  )
  target_link_libraries(unittest_scanner
    ${catkin_LIBRARIES}
  )

  catkin_add_gmock(unittest_udp_client
    test/unit_tests/unittest_udp_client.cpp
  )
  target_link_libraries(unittest_udp_client
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_tenth_degree_conversion
    test/unit_tests/unittest_tenth_degree_conversion.cpp
  )
  target_link_libraries(unittest_tenth_degree_conversion
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_laserscan
    test/unit_tests/unittest_laserscan.cpp
    src/laserscan.cpp
  )
  target_link_libraries(unittest_laserscan
    ${catkin_LIBRARIES}
  )

  catkin_add_gmock(unittest_scanner_controller
    test/unit_tests/unittest_scanner_controller.cpp
    src/scanner_configuration.cpp
    src/start_request.cpp
  )
  target_link_libraries(unittest_scanner_controller
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_scanner_reply_msg
    test/unit_tests/unittest_scanner_reply_msg.cpp
  )
  target_link_libraries(unittest_scanner_reply_msg
    ${catkin_LIBRARIES}
  )

  catkin_add_gtest(unittest_monitoring_frame_msg
    test/unit_tests/unittest_monitoring_frame_msg.cpp
    src/monitoring_frame_msg.cpp
  )
  target_link_libraries(unittest_monitoring_frame_msg
    ${catkin_LIBRARIES}
  )

  #########################
  ##  Integration-Tests  ##
  #########################
  catkin_add_gmock(integrationtest_udp_client
    test/integration_tests/integrationtest_udp_client.cpp
  )
  target_link_libraries(integrationtest_udp_client
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
  )

  add_rostest_gmock(integrationtest_ros_scanner_node
    test/integration_tests/integrationtest_ros_scanner_node.test
    test/integration_tests/integrationtest_ros_scanner_node.cpp
    src/laserscan.cpp
    src/scanner_configuration.cpp
  )
  target_link_libraries(integrationtest_ros_scanner_node
    ${catkin_LIBRARIES}
    ${pilz_testutils_LIBRARIES}
  )

  add_rostest_gtest(integrationtest_ros_parameter_handler
    test/integration_tests/integrationtest_ros_parameter_handler.test
    test/integration_tests/integrationtest_ros_parameter_handler.cpp
    src/ros_parameter_handler.cpp
  )
  target_link_libraries(integrationtest_ros_parameter_handler
    ${catkin_LIBRARIES}
  )

  # generate coverage report using CATKIN_MAKE:
  # catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug psen_scan_v2_coverage (adding -j1 recommended)
  #
  # generate coverage report using CATKIN:
  # catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
  # catkin build
  # catkin build psen_scan_v2 -v --no-deps --catkin-make-args psen_scan_v2_coverage
  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*"
                          "*/${PROJECT_NAME}/src/psen_scan_driver.cpp")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage
      DEPENDS tests
    )
  endif()
endif()
