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
project(psen_scan_v2_standalone)

add_compile_options(-std=c++14)
if (UNIX)
  add_compile_options(-Wall)
  add_compile_options(-Wextra)
  add_compile_options(-Wno-unused-parameter)
  add_compile_options(-Werror)
endif (UNIX)
if (WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4") # Turn of warnings
  add_definitions(-DNOMINMAX)
endif (WIN32)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(console_bridge REQUIRED)
find_package(fmt REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${console_bridge_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_sources
  src/scanner_v2.cpp
  src/laserscan.cpp
  src/monitoring_frame_msg.cpp
  src/start_request.cpp
  src/start_request_serialization.cpp
  src/stop_request_serialization.cpp
  src/scanner_configuration.cpp
  src/monitoring_frame_deserialization.cpp
  src/diagnostics.cpp
  src/scanner_reply_serialization_deserialization.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})

target_link_libraries(${PROJECT_NAME}
  Boost::system
  ${console_bridge_LIBRARIES}
  fmt::fmt
)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_executable(${PROJECT_NAME}_app main.cpp)
target_link_libraries(${PROJECT_NAME}_app
  ${PROJECT_NAME}
)

###########
## Tests ##
###########
if (WIN32)
find_package(GTest CONFIG REQUIRED)
enable_testing()

include_directories(
  test/include
  ${GTEST_INCLUDE_DIRS}
)

ADD_EXECUTABLE(unittest_laserscan_conversions test/unit_tests/unittest_laserscan_conversions.cpp)

TARGET_LINK_LIBRARIES(unittest_laserscan_conversions
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_laserscan_conversions
         COMMAND unittest_laserscan_conversions)

ADD_EXECUTABLE(unittest_laserscan test/unit_tests/unittest_laserscan.cpp)

TARGET_LINK_LIBRARIES(unittest_laserscan
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_laserscan
         COMMAND unittest_laserscan)

ADD_EXECUTABLE(unittest_logging test/unit_tests/unittest_logging.cpp)

TARGET_LINK_LIBRARIES(unittest_logging
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_logging
         COMMAND unittest_logging)


ADD_EXECUTABLE(unittest_monitoring_frame_diagnostic_message test/unit_tests/unittest_monitoring_frame_diagnostic_message.cpp)

TARGET_LINK_LIBRARIES(unittest_monitoring_frame_diagnostic_message
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_monitoring_frame_diagnostic_message
         COMMAND unittest_monitoring_frame_diagnostic_message)

ADD_EXECUTABLE(unittest_monitoring_frame_msg test/unit_tests/unittest_monitoring_frame_msg.cpp)

TARGET_LINK_LIBRARIES(unittest_monitoring_frame_msg
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_monitoring_frame_msg
         COMMAND unittest_monitoring_frame_msg)

ADD_EXECUTABLE(unittest_monitoring_frame_serialization_deserialization test/unit_tests/unittest_monitoring_frame_serialization_deserialization.cpp)

TARGET_LINK_LIBRARIES(unittest_monitoring_frame_serialization_deserialization
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_monitoring_frame_serialization_deserialization
         COMMAND unittest_monitoring_frame_serialization_deserialization)


ADD_EXECUTABLE(unittest_raw_processing test/unit_tests/unittest_raw_processing.cpp)

TARGET_LINK_LIBRARIES(unittest_raw_processing
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_raw_processing
         COMMAND unittest_raw_processing)


ADD_EXECUTABLE(unittest_scan_range test/unit_tests/unittest_scan_range.cpp)

TARGET_LINK_LIBRARIES(unittest_scan_range
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_scan_range
         COMMAND unittest_scan_range)


ADD_EXECUTABLE(unittest_scanner_configuration test/unit_tests/unittest_scanner_configuration.cpp)

TARGET_LINK_LIBRARIES(unittest_scanner_configuration
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_scanner_configuration
         COMMAND unittest_scanner_configuration)


ADD_EXECUTABLE(unittest_scanner_reply_msg test/unit_tests/unittest_scanner_reply_msg.cpp)

TARGET_LINK_LIBRARIES(unittest_scanner_reply_msg
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_scanner_reply_msg
         COMMAND unittest_scanner_reply_msg)


ADD_EXECUTABLE(unittest_start_request test/unit_tests/unittest_start_request.cpp)

TARGET_LINK_LIBRARIES(unittest_start_request
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_start_request
         COMMAND unittest_start_request)


ADD_EXECUTABLE(unittest_stop_request test/unit_tests/unittest_stop_request.cpp)

TARGET_LINK_LIBRARIES(unittest_stop_request
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_stop_request
        COMMAND unittest_stop_request)


ADD_EXECUTABLE(unittest_tenth_degree_conversion test/unit_tests/unittest_tenth_degree_conversion.cpp)

TARGET_LINK_LIBRARIES(unittest_tenth_degree_conversion
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_tenth_degree_conversion
        COMMAND unittest_tenth_degree_conversion)


ADD_EXECUTABLE(unittest_tenth_of_degree test/unit_tests/unittest_tenth_of_degree.cpp)

TARGET_LINK_LIBRARIES(unittest_tenth_of_degree
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_tenth_of_degree
        COMMAND unittest_tenth_of_degree)


ADD_EXECUTABLE(unittest_udp_client test/unit_tests/unittest_udp_client.cpp)

TARGET_LINK_LIBRARIES(unittest_udp_client
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    GTest::gmock GTest::gtest GTest::gmock_main GTest::gtest_main
)

ADD_TEST(NAME unittest_udp_client
        COMMAND unittest_udp_client)

endif (WIN32)
