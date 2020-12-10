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

#include "psen_scan_v2_standalone/scanner_reply_serialization_deserialization.h"

#include <iostream>
#include <algorithm>
#include <cassert>

namespace psen_scan_v2_standalone
{
template <typename T>
inline void processBytes(boost::crc_32_type& crc_32, const T& data)
{
  crc_32.process_bytes(&data, sizeof(T));
}

scanner_reply::RawType scanner_reply::serialize(const uint32_t op_code, const uint32_t res_code)
{
  std::ostringstream os;

  const uint32_t reserved{ 0 };

  boost::crc_32_type crc;
  crc.process_bytes(&reserved, sizeof(reserved));
  crc.process_bytes(&op_code, sizeof(op_code));
  crc.process_bytes(&res_code, sizeof(res_code));

  raw_processing::write(os, static_cast<uint32_t>(crc.checksum()));
  raw_processing::write(os, reserved);
  raw_processing::write(os, op_code);
  raw_processing::write(os, res_code);

  // TODO check limits
  const std::string data_str(os.str());
  assert(data_str.length() == scanner_reply::Message::SIZE && "Message data of start reply has not the expected size");

  scanner_reply::RawType ret_val{};
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

scanner_reply::RawType scanner_reply::serialize(const Message& reply)
{
  return serialize(static_cast<uint32_t>(reply.type()), static_cast<uint32_t>(reply.result()));
}
}  // namespace psen_scan_v2_standalone
