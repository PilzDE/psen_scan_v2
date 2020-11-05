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
#ifndef PSEN_SCAN_V2_SCANNER_REPLY_SERIALIZATION_DESERIALIZATION_H
#define PSEN_SCAN_V2_SCANNER_REPLY_SERIALIZATION_DESERIALIZATION_H

#include <cstdint>
#include <sstream>

#include <boost/crc.hpp>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/scanner_reply_msg.h"

namespace psen_scan_v2
{
namespace scanner_reply
{
class CRCMismatch : public std::runtime_error
{
public:
  CRCMismatch(const std::string& msg = "CRC did not match!");
};

template <typename T>
Message deserialize(const T& data)
{
  const T tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), Message::SIZE));

  uint32_t crc;
  uint32_t reserved;
  uint32_t opcode;
  uint32_t res_code;

  raw_processing::read(is, crc);
  raw_processing::read(is, reserved);
  raw_processing::read(is, opcode);
  raw_processing::read(is, res_code);

  boost::crc_32_type crc_checked;
  crc_checked.process_bytes(&reserved, sizeof(reserved));
  crc_checked.process_bytes(&opcode, sizeof(opcode));
  crc_checked.process_bytes(&res_code, sizeof(res_code));

  if (crc != crc_checked.checksum())
  {
    throw scanner_reply::CRCMismatch();
  }

  return Message(Message::convertToReplyType(opcode), Message::convertToOperationResult(res_code));
}

using RawType = FixedSizeRawData<Message::SIZE>;
RawType serialize(const Message& reply);
RawType serialize(const uint32_t op_code, const uint32_t res_code);

inline CRCMismatch::CRCMismatch(const std::string& msg) : std::runtime_error(msg)
{
}
}  // namespace scanner_reply
}  // namespace psen_scan_v2
#endif  // PSEN_SCAN_V2_SCANNER_REPLY_SERIALIZATION_DESERIALIZATION_H
