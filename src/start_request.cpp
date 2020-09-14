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

  DynamicSizeRawData raw_data{ toRawData() };

  result.process_bytes(&raw_data.at(sizeof(crc_)), raw_data.size() - sizeof(crc_));

  return result.checksum();
}

DynamicSizeRawData StartRequest::toRawData() const
{
  std::ostringstream os;

  os.write((char*)&crc_, sizeof(crc_));
  os.write((char*)&seq_number_, sizeof(seq_number_));
  os.write((char*)&RESERVED_, sizeof(RESERVED_));
  os.write((char*)&OPCODE_, sizeof(OPCODE_));

  uint32_t host_ip_big_endian = htobe32(host_ip_);
  os.write((char*)&host_ip_big_endian, sizeof(host_ip_big_endian));

  os.write((char*)&host_udp_port_data_, sizeof(host_udp_port_data_));
  os.write((char*)&device_enabled_, sizeof(device_enabled_));
  os.write((char*)&intensity_enabled_, sizeof(intensity_enabled_));
  os.write((char*)&point_in_safety_enabled_, sizeof(point_in_safety_enabled_));
  os.write((char*)&active_zone_set_enabled_, sizeof(active_zone_set_enabled_));
  os.write((char*)&io_pin_enabled_, sizeof(io_pin_enabled_));
  os.write((char*)&scan_counter_enabled_, sizeof(scan_counter_enabled_));
  os.write((char*)&speed_encoder_enabled_, sizeof(speed_encoder_enabled_));
  os.write((char*)&diagnostics_enabled_, sizeof(diagnostics_enabled_));

  uint16_t start_angle{ radToTenthDegree(master_.getStartAngle()) };
  uint16_t end_angle{ radToTenthDegree(master_.getEndAngle()) };
  uint16_t resolution{ radToTenthDegree(master_.getResolution()) };
  os.write((char*)&start_angle, sizeof(start_angle));
  os.write((char*)&end_angle, sizeof(end_angle));
  os.write((char*)&resolution, sizeof(resolution));

  for (const auto& slave : slaves_)
  {
    uint16_t slave_start_angle{ radToTenthDegree(slave.getStartAngle()) };
    uint16_t slave_end_angle{ radToTenthDegree(slave.getEndAngle()) };
    uint16_t slave_resolution{ radToTenthDegree(slave.getResolution()) };
    os.write((char*)&slave_start_angle, sizeof(slave_start_angle));
    os.write((char*)&slave_end_angle, sizeof(slave_end_angle));
    os.write((char*)&slave_resolution, sizeof(slave_resolution));
  }

  std::string data_str(os.str());

  DynamicSizeRawData raw_data;
  raw_data.reserve(data_str.length());
  assert(data_str.length() == START_REQUEST_SIZE && "Message data of start request has not the expected size");
  std::copy(data_str.begin(), data_str.end(), std::back_inserter(raw_data));

  return raw_data;
}

}  // namespace psen_scan_v2
