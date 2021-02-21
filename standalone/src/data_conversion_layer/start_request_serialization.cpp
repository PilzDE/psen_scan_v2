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

#include <string>
#include <cassert>
#include <iostream>

#ifdef _WIN32
#include <Windows.h>
#endif

#include <boost/crc.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/start_request.h"
#include "psen_scan_v2_standalone/data_conversion_layer/start_request_serialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace start_request
{
static constexpr std::size_t SIZE{ 58 };  // See protocol description
static constexpr uint64_t RESERVED{ 0 };
static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0 };

static const uint32_t OPCODE{ 0x35 };
}  // namespace start_request

uint32_t calculateCRC(const data_conversion_layer::RawData& data)
{
  boost::crc_32_type crc;
  crc.process_bytes(&data.at(0), data.size());
  return static_cast<uint32_t>(crc.checksum());
}

RawData data_conversion_layer::start_request::serialize(const data_conversion_layer::start_request::Message& msg,
                                                        const uint32_t& seq_number)
{
  std::ostringstream os;

  raw_processing::write(os, seq_number);
  raw_processing::write(os, data_conversion_layer::start_request::RESERVED);
  raw_processing::write(os, data_conversion_layer::start_request::OPCODE);

/**< Byte order: big endian */
#ifdef __linux__
  const uint32_t host_ip_big_endian = htobe32(msg.host_ip_);
#endif

#ifdef _WIN32
  const uint32_t host_ip_big_endian = _byteswap_ulong(msg.host_ip_);
#endif

  raw_processing::write(os, host_ip_big_endian);

  /**< Byte order: big endian */
  raw_processing::write(os, msg.host_udp_port_data_);

  /**< The following 'enable' fields are a 1-byte mask each.
   * Only the last 4 bits (little endian) are used, each of which represents a device.
   * For example, (1000) only enables the Master device, while (1010) enables both the Master
   * and the second Slave device.
   */

  const uint8_t device_enabled{ 0b00001000 };
  const uint8_t intensity_enabled{ 0b00001000 };
  const uint8_t point_in_safety_enabled{ 0 };
  const uint8_t active_zone_set_enabled{ 0 };
  const uint8_t io_pin_enabled{ 0 };
  const uint8_t scan_counter_enabled{ 0b00001000 };
  const uint8_t speed_encoder_enabled{ 0 }; /**< 0000000bin disabled, 00001111bin enabled.*/
  const uint8_t diagnostics_enabled{ static_cast<uint8_t>(
      msg.master_device_settings_.isDiagnosticsEnabled() ? 0b00001000 : 0b00000000) };

  raw_processing::write(os, device_enabled);
  raw_processing::write(os, intensity_enabled);
  raw_processing::write(os, point_in_safety_enabled);
  raw_processing::write(os, active_zone_set_enabled);
  raw_processing::write(os, io_pin_enabled);
  raw_processing::write(os, scan_counter_enabled);
  raw_processing::write(os, speed_encoder_enabled);
  raw_processing::write(os, diagnostics_enabled);

  raw_processing::write(os, msg.master_.getScanRange().getStart().value());
  raw_processing::write(os, msg.master_.getScanRange().getEnd().value());
  raw_processing::write(os, msg.master_.getResolution().value());

  for (const auto& slave : msg.slaves_)
  {
    raw_processing::write(os, slave.getScanRange().getStart().value());
    raw_processing::write(os, slave.getScanRange().getEnd().value());
    raw_processing::write(os, slave.getResolution().value());
  }

  const std::string raw_data_as_str{ os.str() };
  const data_conversion_layer::RawData raw_data(raw_data_as_str.cbegin(), raw_data_as_str.cend());

  std::ostringstream os_crc;
  raw_processing::write(os_crc, calculateCRC(raw_data));

  std::string raw_data_with_crc_str(os_crc.str() + os.str());
  data_conversion_layer::RawData raw_data_with_crc{ raw_data_with_crc_str.cbegin(), raw_data_with_crc_str.cend() };

  assert(raw_data_with_crc.size() == SIZE && "Message data of start request has not the expected size");

  return raw_data_with_crc;
}

RawData data_conversion_layer::start_request::serialize(const data_conversion_layer::start_request::Message& msg)
{
  return serialize(msg, data_conversion_layer::start_request::DEFAULT_SEQ_NUMBER);
}
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
