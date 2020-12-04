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
project(psen_scan_v2_standalone)

add_compile_options(-std=c++14)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(console_bridge REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  fmt_include
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
  src/fmt/format.cc
  src/fmt/os.cc
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources})
target_link_libraries(${PROJECT_NAME}
  ${console_bridge_LIBRARIES}
  ${rosfmt_LIBRARIES}
)


find_package(catkin REQUIRED COMPONENTS
  rosconsole_bridge
  roscpp
  sensor_msgs
  rosfmt
)
