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
#ifndef PSEN_SCAN_V2_EVENTS_H
#define PSEN_SCAN_V2_EVENTS_H

#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
namespace scanner_protocol
{
namespace scanner_events
{
//! @brief User requests scanner to start.
class StartRequest
{
};

//! @brief User requests scanner to stop.
class StopRequest
{
};

//! @brief Timeout while waiting for scanner device to start.
class StartTimeout
{
};

//! @brief Received Start- or Stop-Reply message from scanner device.
class RawReplyReceived
{
public:
  RawReplyReceived(const MaxSizeRawData& data, const std::size_t& num_bytes) : data_(data), num_bytes_(num_bytes)
  {
  }

public:
  const MaxSizeRawData data_;
  const std::size_t num_bytes_;
};

//! @brief Triggered whenever the receiving of a reply message failes.
class ReplyReceiveError
{
};

//! @brief Received monitoring frame from scanner device.
class RawMonitoringFrameReceived
{
public:
  RawMonitoringFrameReceived(const MaxSizeRawData& data, const std::size_t& num_bytes)
    : data_(data), num_bytes_(num_bytes)
  {
  }

public:
  const MaxSizeRawData data_;
  const std::size_t num_bytes_;
};

//! @brief Triggered whenever the receiving of a monitoring frame failes.
class MonitoringFrameReceivedError
{
};

}  // namespace scanner_events
}  // namespace scanner_protocol
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_EVENTS_H
