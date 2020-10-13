// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_V2_LOGGING_H
#define PSEN_SCAN_V2_LOGGING_H

#include <chrono>
#include <console_bridge/console.h>
#include <sstream>
#include <fmt/core.h>

#define PSENSCAN_LOG(name, file, line, level, ...)                                                                     \
  {                                                                                                                    \
    console_bridge::getOutputHandler()->log(fmt::format("{}: {}", name, fmt::format(__VA_ARGS__)), level, file, line); \
  }

#define PSENSCAN_LOG_THROTTLE(period, name, file, line, level, ...)                                                    \
  {                                                                                                                    \
    static std::chrono::system_clock::time_point throttle_last_hit;                                                    \
    auto throttle_now = std::chrono::system_clock::now();                                                              \
    if (throttle_last_hit + std::chrono::duration<double>(period) < throttle_now)                                      \
    {                                                                                                                  \
      throttle_last_hit = throttle_now;                                                                                \
      PSENSCAN_LOG(name, file, line, level, __VA_ARGS__);                                                              \
    }                                                                                                                  \
  }

using namespace console_bridge;

#define PSENSCAN_ERROR(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_ERROR, __VA_ARGS__)
#define PSENSCAN_INFO(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_INFO, __VA_ARGS__)
#define PSENSCAN_WARN(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_WARN, __VA_ARGS__)
#define PSENSCAN_DEBUG(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_DEBUG, __VA_ARGS__)

#define PSENSCAN_ERROR_THROTTLE(period, name, ...)                                                                     \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_ERROR, __VA_ARGS__)
#define PSENSCAN_INFO_THROTTLE(period, name, ...)                                                                      \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_INFO, __VA_ARGS__)
#define PSENSCAN_WARN_THROTTLE(period, name, ...)                                                                      \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_WARN, __VA_ARGS__)
#define PSENSCAN_DEBUG_THROTTLE(period, name, ...)                                                                     \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_DEBUG, __VA_ARGS__)

#endif  // PSEN_SCAN_V2_LOGGING_H