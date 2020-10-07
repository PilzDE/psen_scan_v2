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

#include <cassert>
#include <algorithm>
#include <sstream>
#include <string>
#include <iostream>

#include <boost/crc.hpp>

#include "psen_scan_v2/stop_request.h"
#include "psen_scan_v2/raw_data_creation.h"
#include "psen_scan_v2/raw_processing.h"

namespace psen_scan_v2
{
std::ostringstream& StopRequest::processMember(std::ostringstream& os) const
{
  auto reserved{ RESERVED_ };
  raw_processing::write(os, reserved);

  auto op_code{ OPCODE_ };
  raw_processing::write(os, op_code);

  return os;
}

uint32_t StopRequest::calcCrc() const
{
  std::ostringstream os;
  return psen_scan_v2::calcCrc(processMember(os));
}

DynamicSizeRawData StopRequest::serialize() const
{
  std::ostringstream os;
  uint32_t crc{ calcCrc() };
  raw_processing::write(os, crc);
  processMember(os);
  return raw_processing::serialize<DynamicSizeRawData>(os);
}

}  // namespace psen_scan_v2
