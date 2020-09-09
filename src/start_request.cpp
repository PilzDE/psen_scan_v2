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

#include "psen_scan_v2/tenth_degree_conversion.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/degree_to_rad.h"

namespace psen_scan_v2
{
static constexpr double MASTER_RESOLUTION_RAD{ degreeToRad(1.) };

StartRequest::StartRequest(const ScannerConfiguration& scanner_configuration, const uint32_t& seq_number)
  : seq_number_(seq_number)
  , host_ip_(scanner_configuration.hostIp())
  , host_udp_port_data_(scanner_configuration.hostUDPPortData())  // Write is deduced by the scanner
  , master_(scanner_configuration.startAngle(), scanner_configuration.endAngle(), MASTER_RESOLUTION_RAD)
{
  crc_ = getCRC();
}

uint32_t StartRequest::getCRC() const
{
  boost::crc_32_type result;

  std::vector<char> raw_data = toRawType();

  result.process_bytes(&raw_data.at(sizeof(crc_)), raw_data.size() - sizeof(crc_));

  return result.checksum();
}

StartRequest::RawType StartRequest::toRawType() const
{
  std::ostringstream os;

  write<uint32_t>(os, crc_);
  write<uint32_t>(os, seq_number_);
  write<uint64_t>(os, RESERVED_);
  write<uint32_t>(os, OPCODE_);

  uint32_t host_ip_big_endian = htobe32(host_ip_);
  write<uint32_t>(os, host_ip_big_endian);

  write<uint16_t>(os, host_udp_port_data_);
  write<uint8_t>(os, device_enabled_);
  write<uint8_t>(os, intensity_enabled_);
  write<uint8_t>(os, point_in_safety_enabled_);
  write<uint8_t>(os, active_zone_set_enabled_);
  write<uint8_t>(os, io_pin_enabled_);
  write<uint8_t>(os, scan_counter_enabled_);
  write<uint8_t>(os, speed_encoder_enabled_);
  write<uint8_t>(os, diagnostics_enabled_);

  uint16_t start_angle{ radToTenthDegree(master_.getStartAngle()) };
  uint16_t end_angle{ radToTenthDegree(master_.getEndAngle()) };
  uint16_t resolution{ radToTenthDegree(master_.getResolution()) };
  write<uint16_t>(os, start_angle);
  write<uint16_t>(os, end_angle);
  write<uint16_t>(os, resolution);

  for (const auto& slave : slaves_)
  {
    uint16_t slave_start_angle{ radToTenthDegree(slave.getStartAngle()) };
    uint16_t slave_end_angle{ radToTenthDegree(slave.getEndAngle()) };
    uint16_t slave_resolution{ radToTenthDegree(slave.getResolution()) };
    write<uint16_t>(os, slave_start_angle);
    write<uint16_t>(os, slave_end_angle);
    write<uint16_t>(os, slave_resolution);
  }

  std::string data_str(os.str());

  std::vector<char> raw_data;
  raw_data.reserve(data_str.length());
  assert(data_str.length() == START_REQUEST_SIZE && "Message data of start request has not the expected size");
  std::copy(data_str.begin(), data_str.end(), std::back_inserter(raw_data));

  return raw_data;
}

template <typename T>
void StartRequest::write(std::ostringstream& os, const T& data) const
{
  os.write((char*)&data, sizeof(T));
  return;
}

}  // namespace psen_scan_v2
