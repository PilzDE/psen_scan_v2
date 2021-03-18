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

#include "psen_scan_v2_standalone/data_conversion_layer/stop_request_serialization.h"

#include <iostream>

#include <boost/crc.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
RawData data_conversion_layer::stop_request::serialize()
{
  std::ostringstream os;

  boost::crc_32_type crc;
  crc.process_bytes(&stop_request::RESERVED, sizeof(data_conversion_layer::stop_request::RESERVED));
  crc.process_bytes(&stop_request::OPCODE, sizeof(data_conversion_layer::stop_request::OPCODE));

  raw_processing::write(os, static_cast<uint32_t>(crc.checksum()));
  raw_processing::write(os, data_conversion_layer::stop_request::RESERVED);
  raw_processing::write(os, data_conversion_layer::stop_request::OPCODE);

  return raw_processing::toArray<RawData>(os);
}
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
