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

#ifndef PSEN_SCAN_V2_REPLY_MSG_FROM_SCANNER_H
#define PSEN_SCAN_V2_REPLY_MSG_FROM_SCANNER_H

#include <cstdint>
#include <cassert>
#include <array>
#include <sstream>

#include <boost/crc.hpp>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/decode_exception.h"

namespace psen_scan_v2
{
enum class ReplyMsgFromScannerType
{
  Start,
  Unknown
};

static constexpr std::size_t REPLY_MSG_FROM_SCANNER_SIZE = 16;  // See protocol description

class ReplyMsgFromScanner
{
public:
  static ReplyMsgFromScanner fromRawData(const RawScannerData& data);

public:
  ReplyMsgFromScanner(const uint32_t op_code, const uint32_t res_code);

public:
  ReplyMsgFromScannerType type() const;

public:
  static uint32_t getStartOpCode();
  static uint32_t calcCRC(const ReplyMsgFromScanner& msg);

  using RawType = std::array<char, REPLY_MSG_FROM_SCANNER_SIZE>;
  RawType toCharArray();

private:
  ReplyMsgFromScanner() = delete;

private:
  //! A CRC32 of all the following fields.
  uint32_t crc_{ 0 };
  uint32_t reserved_{ 0 };
  //! Operation Code (START 0x35, STOP 0x36).
  uint32_t opcode_{ 0 };
  //! Operation result.
  //! If the message is accepted, the returned value is 0x00.
  //! If the message is refused, the returned value is 0xEB.
  //! If the CRC is not correct, the device will not send any message.
  uint32_t res_code_{ 0 };

private:
  static constexpr uint32_t OPCODE_START{ 0x35 };
};

inline uint32_t ReplyMsgFromScanner::calcCRC(const ReplyMsgFromScanner& msg)
{
  boost::crc_32_type result;
  // Read all data except the field of the sent crc at the beginning according to:
  // Reference Guide Rev. A – November 2019 Page 14
  result.process_bytes(&(msg.reserved_), sizeof(ReplyMsgFromScanner::reserved_));
  result.process_bytes(&(msg.opcode_), sizeof(ReplyMsgFromScanner::opcode_));
  result.process_bytes(&(msg.res_code_), sizeof(ReplyMsgFromScanner::res_code_));
  return result.checksum();
}

inline uint32_t ReplyMsgFromScanner::getStartOpCode()
{
  return OPCODE_START;
}

inline ReplyMsgFromScanner::ReplyMsgFromScanner(const uint32_t op_code, const uint32_t res_code)
  : opcode_(op_code), res_code_(res_code)
{
  crc_ = calcCRC(*this);
}

inline ReplyMsgFromScanner ReplyMsgFromScanner::fromRawData(const RawScannerData& data)
{
  ReplyMsgFromScanner msg{ 0, 0 };

  // Alternatives for transformation:
  // typedef boost::iostreams::basic_array_source<char> Device;
  // boost::iostreams::stream<Device> stream((char*)&data, sizeof(DataReply::MemoryFormat));

  std::istringstream stream(std::string((char*)&data, REPLY_MSG_FROM_SCANNER_SIZE));

  stream.read((char*)&msg.crc_, sizeof(ReplyMsgFromScanner::crc_));
  stream.read((char*)&msg.reserved_, sizeof(ReplyMsgFromScanner::reserved_));
  stream.read((char*)&msg.opcode_, sizeof(ReplyMsgFromScanner::opcode_));
  stream.read((char*)&msg.res_code_, sizeof(ReplyMsgFromScanner::res_code_));

  if (msg.crc_ != calcCRC(msg))
  {
    throw DecodeCRCMismatchException();
  }

  return msg;
}

inline ReplyMsgFromScannerType ReplyMsgFromScanner::type() const
{
  if (opcode_ == OPCODE_START)
  {
    return ReplyMsgFromScannerType::Start;
  }
  return ReplyMsgFromScannerType::Unknown;
}

inline ReplyMsgFromScanner::RawType ReplyMsgFromScanner::toCharArray()
{
  std::ostringstream os;

  uint32_t crc{ calcCRC(*this) };
  os.write((char*)&crc, sizeof(uint32_t));
  os.write((char*)&reserved_, sizeof(uint32_t));
  os.write((char*)&opcode_, sizeof(uint32_t));
  os.write((char*)&res_code_, sizeof(uint32_t));

  ReplyMsgFromScanner::RawType ret_val{};

  // TODO check limits
  std::string data_str(os.str());
  assert(data_str.length() == REPLY_MSG_FROM_SCANNER_SIZE && "Message data of start reply has not the expected size");
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_REPLY_MSG_FROM_SCANNER_H
