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
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

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
  src/data_conversion_layer/monitoring_frame_msg.cpp
  src/data_conversion_layer/start_request.cpp
  src/data_conversion_layer/start_request_serialization.cpp
  src/data_conversion_layer/stop_request_serialization.cpp
  src/scanner_configuration.cpp
  src/data_conversion_layer/monitoring_frame_deserialization.cpp
  src/data_conversion_layer/diagnostics.cpp
  src/data_conversion_layer/scanner_reply_serialization_deserialization.cpp
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
