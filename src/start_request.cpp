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

#include <arpa/inet.h>
#include <boost/crc.hpp>
#include <endian.h>

#include <algorithm>
#include <sstream>
#include <string>
#include <iostream>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/raw_processing.h"

namespace psen_scan_v2
{
static constexpr double MASTER_RESOLUTION_RAD{ degreeToRadian(0.1) };

StartRequest::StartRequest(const ScannerConfiguration& scanner_configuration, const uint32_t& seq_number)
  : seq_number_(seq_number)
  , host_ip_(scanner_configuration.hostIp())
  , host_udp_port_data_(scanner_configuration.hostUDPPortData())  // Write is deduced by the scanner
  , master_(scanner_configuration.scanRange(), MASTER_RESOLUTION_RAD)
{
  crc_ = getCRC();
}

uint32_t StartRequest::getCRC() const
{
  boost::crc_32_type result;

  DynamicSizeRawData raw_data{ serialize() };

  result.process_bytes(&raw_data.at(sizeof(crc_)), raw_data.size() - sizeof(crc_));

  return result.checksum();
}

DynamicSizeRawData StartRequest::serialize() const
{
  std::ostringstream os;

  raw_processing::write(os, crc_);
  raw_processing::write(os, seq_number_);
  raw_processing::write(os, RESERVED_);
  raw_processing::write(os, OPCODE_);

  uint32_t host_ip_big_endian = htobe32(host_ip_);
  raw_processing::write(os, host_ip_big_endian);

  raw_processing::write(os, host_udp_port_data_);
  raw_processing::write(os, device_enabled_);
  raw_processing::write(os, intensity_enabled_);
  raw_processing::write(os, point_in_safety_enabled_);
  raw_processing::write(os, active_zone_set_enabled_);
  raw_processing::write(os, io_pin_enabled_);
  raw_processing::write(os, scan_counter_enabled_);
  raw_processing::write(os, speed_encoder_enabled_);
  raw_processing::write(os, diagnostics_enabled_);

  uint16_t start_angle{ master_.getScanRange().getStart().value() };
  uint16_t end_angle{ master_.getScanRange().getEnd().value() };
  uint16_t resolution{ radToTenthDegree(master_.getResolution()) };
  raw_processing::write(os, start_angle);
  raw_processing::write(os, end_angle);
  raw_processing::write(os, resolution);

  for (const auto& slave : slaves_)
  {
    uint16_t slave_start_angle{ slave.getScanRange().getStart().value() };
    uint16_t slave_end_angle{ slave.getScanRange().getEnd().value() };
    uint16_t slave_resolution{ radToTenthDegree(slave.getResolution()) };
    raw_processing::write(os, slave_start_angle);
    raw_processing::write(os, slave_end_angle);
    raw_processing::write(os, slave_resolution);
  }

  std::string data_str(os.str());

  DynamicSizeRawData raw_data;
  raw_data.reserve(data_str.length());
  assert(data_str.length() == START_REQUEST_SIZE && "Message data of start request has not the expected size");
  std::copy(data_str.begin(), data_str.end(), std::back_inserter(raw_data));

  return raw_data;
}
}  // namespace psen_scan_v2
