// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_LOGGING_H
#define PSEN_SCAN_V2_STANDALONE_LOGGING_H

#include <chrono>
#include <sstream>

#include <fmt/format.h>
#include <fmt/ostream.h>

#ifdef _ROS_BUILD_
#include <rcutils/logging_macros.h>
#define PSENSCAN_LOG(name, file, line, level, ...)                                                                     \
  RCUTILS_LOG_COND_NAMED(RCUTILS_LOG_SEVERITY_##level,                                                                 \
                         RCUTILS_LOG_CONDITION_EMPTY,                                                                  \
                         RCUTILS_LOG_CONDITION_EMPTY,                                                                  \
                         name,                                                                                         \
                         fmt::format(__VA_ARGS__).c_str())
#else
#include <iostream>
// ToDo Either use console_bridge or extend own implementation, see
// https://github.com/ros/console_bridge/blob/master/src/console.cpp#L109
#define PSENSCAN_LOG(name, file, line, level, ...)                                                                     \
  std::cout << fmt::format("{}: {}: {}", #level, name, fmt::format(__VA_ARGS__)) << std::endl;
#endif

#define PSENSCAN_LOG_ONCE(name, file, line, level, ...)                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    static bool already_logged = false;                                                                                \
    if (!already_logged)                                                                                               \
    {                                                                                                                  \
      PSENSCAN_LOG(name, file, line, level, __VA_ARGS__);                                                              \
      already_logged = true;                                                                                           \
    }                                                                                                                  \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define PSENSCAN_LOG_THROTTLE(period, name, file, line, level, ...)                                                    \
  PSENSCAN_LOG_THROTTLE_INTERNAL(std::chrono::system_clock::now(), period, name, file, line, level, __VA_ARGS__)

#define PSENSCAN_LOG_THROTTLE_INTERNAL(now, period, name, file, line, level, ...)                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    static std::chrono::system_clock::time_point throttle_last_hit;                                                    \
    auto throttle_now = now;                                                                                           \
    if (throttle_last_hit + std::chrono::duration<double>(period) < throttle_now)                                      \
    {                                                                                                                  \
      throttle_last_hit = throttle_now;                                                                                \
      PSENSCAN_LOG(name, file, line, level, __VA_ARGS__);                                                              \
    }                                                                                                                  \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define PSENSCAN_ERROR(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, ERROR, __VA_ARGS__)
#define PSENSCAN_INFO(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, INFO, __VA_ARGS__)
#define PSENSCAN_WARN(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, WARN, __VA_ARGS__)
#define PSENSCAN_DEBUG(name, ...) PSENSCAN_LOG(name, __FILE__, __LINE__, DEBUG, __VA_ARGS__)

#define PSENSCAN_ERROR_THROTTLE_INTERNAL(now, period, name, ...)                                                       \
  PSENSCAN_LOG_THROTTLE_INTERNAL(now, period, name, __FILE__, __LINE__, ERROR, __VA_ARGS__)
#define PSENSCAN_INFO_THROTTLE_INTERNAL(now, period, name, ...)                                                        \
  PSENSCAN_LOG_THROTTLE_INTERNAL(now, period, name, __FILE__, __LINE__, INFO, __VA_ARGS__)
#define PSENSCAN_WARN_THROTTLE_INTERNAL(now, period, name, ...)                                                        \
  PSENSCAN_LOG_THROTTLE_INTERNAL(now, period, name, __FILE__, __LINE__, WARN, __VA_ARGS__)
#define PSENSCAN_DEBUG_THROTTLE_INTERNAL(now, period, name, ...)                                                       \
  PSENSCAN_LOG_THROTTLE_INTERNAL(now, period, name, __FILE__, __LINE__, DEBUG, __VA_ARGS__)

#define PSENSCAN_ERROR_THROTTLE(period, name, ...)                                                                     \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, ERROR, __VA_ARGS__)
#define PSENSCAN_INFO_THROTTLE(period, name, ...)                                                                      \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, INFO, __VA_ARGS__)
#define PSENSCAN_WARN_THROTTLE(period, name, ...)                                                                      \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, WARN, __VA_ARGS__)
#define PSENSCAN_DEBUG_THROTTLE(period, name, ...)                                                                     \
  PSENSCAN_LOG_THROTTLE(period, name, __FILE__, __LINE__, DEBUG, __VA_ARGS__)

#define PSENSCAN_ERROR_ONCE(name, ...) PSENSCAN_LOG_ONCE(name, __FILE__, __LINE__, ERROR, __VA_ARGS__)
#define PSENSCAN_INFO_ONCE(name, ...) PSENSCAN_LOG_ONCE(name, __FILE__, __LINE__, INFO, __VA_ARGS__)
#define PSENSCAN_WARN_ONCE(name, ...) PSENSCAN_LOG_ONCE(name, __FILE__, __LINE__, WARN, __VA_ARGS__)
#define PSENSCAN_DEBUG_ONCE(name, ...) PSENSCAN_LOG_ONCE(name, __FILE__, __LINE__, DEBUG, __VA_ARGS__)

#endif  // PSEN_SCAN_V2_STANDALONE_LOGGING_H