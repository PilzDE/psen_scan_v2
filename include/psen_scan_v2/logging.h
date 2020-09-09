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

#include <console_bridge/console.h>
#include <sstream>

#define PSENSCAN_LOG(name, file, line, level, args)                                                                    \
  {                                                                                                                    \
    std::stringstream ss;                                                                                              \
    ss << name << ": " << args;                                                                                        \
    console_bridge::getOutputHandler()->log(ss.str(), level, file, line);                                              \
  }

using namespace console_bridge;

#define PSENSCAN_ERROR(name, args) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_ERROR, args)
#define PSENSCAN_INFO(name, args) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_INFO, args)
#define PSENSCAN_WARN(name, args) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_WARN, args)
#define PSENSCAN_DEBUG(name, args) PSENSCAN_LOG(name, __FILE__, __LINE__, CONSOLE_BRIDGE_LOG_DEBUG, args)

#endif  // PSEN_SCAN_V2_LOGGING_H