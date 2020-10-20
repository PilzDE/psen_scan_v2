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

#ifndef PSEN_SCAN_V2_FUNCTION_POINTERS_H
#define PSEN_SCAN_V2_FUNCTION_POINTERS_H

#include <functional>

#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/laserscan.h"

namespace psen_scan_v2
{
using SendRequestCallback = std::function<void()>;
using ReplyCallback = std::function<void()>;
using MonitoringFrameCallback = std::function<void(const monitoring_frame::Message&)>;
using ErrorCallback = std::function<void(const std::string&)>;
using LaserScanCallback = std::function<void(const LaserScan&)>;
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_FUNCTION_POINTERS_H
