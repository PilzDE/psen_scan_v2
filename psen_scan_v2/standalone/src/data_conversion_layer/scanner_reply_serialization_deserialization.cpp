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

#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_serialization_deserialization.h"

#include <algorithm>
#include <cassert>
#include <boost/crc.hpp>
#include <sstream>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
scanner_reply::CRCMismatch::CRCMismatch(const std::string& msg) : std::runtime_error(msg)
{
}

RawData data_conversion_layer::scanner_reply::serialize(const uint32_t op_code, const uint32_t res_code)
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
  assert(data_str.length() == data_conversion_layer::scanner_reply::Message::SIZE &&
         "Message data of start reply has not the expected size");
  return data_conversion_layer::RawData(data_str.cbegin(), data_str.cend());
}

RawData data_conversion_layer::scanner_reply::serialize(const Message& reply)
{
  return serialize(static_cast<uint32_t>(reply.type()), static_cast<uint32_t>(reply.result()));
}

scanner_reply::Message data_conversion_layer::scanner_reply::deserialize(const data_conversion_layer::RawData& data)
{
  std::istringstream is(std::string(data.data(), Message::SIZE));

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
    throw data_conversion_layer::scanner_reply::CRCMismatch();
  }

  return Message(Message::convertToReplyType(opcode), Message::convertToOperationResult(res_code));
}

}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
