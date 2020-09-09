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

#ifndef PSEN_SCAN_REPLY_MSG_FROM_SCANNER_H
#define PSEN_SCAN_REPLY_MSG_FROM_SCANNER_H

#include <cstdint>
#include <cassert>
#include <array>
#include <sstream>

#include <boost/crc.hpp>

#include "psen_scan/raw_scanner_data.h"
#include "psen_scan/decode_exception.h"

namespace psen_scan
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

private:
  template <typename T>
  static void read(std::istringstream& is, const T& data);

public:
  ReplyMsgFromScanner(const uint32_t op_code, const uint32_t res_code);
  ReplyMsgFromScannerType type() const;
  static uint32_t getStartOpCode();
  static uint32_t calcCRC(const ReplyMsgFromScanner& msg);

private:
  template <typename T>
  static void process_bytes(boost::crc_32_type& crc_32, const T& data);

public:
  using RawType = std::array<char, REPLY_MSG_FROM_SCANNER_SIZE>;
  RawType toCharArray();

private:
  template <typename T>
  void write(std::ostringstream& os, const T& data) const;

  ReplyMsgFromScanner() = delete;

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

  static constexpr uint32_t OPCODE_START{ 0x35 };
};

inline uint32_t ReplyMsgFromScanner::calcCRC(const ReplyMsgFromScanner& msg)
{
  boost::crc_32_type result;
  // Read all data except the field of the sent crc at the beginning according to:
  // Reference Guide Rev. A – November 2019 Page 14
  process_bytes<uint32_t>(result, msg.reserved_);
  process_bytes<uint32_t>(result, msg.opcode_);
  process_bytes<uint32_t>(result, msg.res_code_);
  return result.checksum();
}

template <typename T>
inline void ReplyMsgFromScanner::process_bytes(boost::crc_32_type& crc_32, const T& data)
{
  crc_32.process_bytes(&(data), sizeof(T));
  return;
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

  std::istringstream is(std::string((char*)&data, REPLY_MSG_FROM_SCANNER_SIZE));

  read<uint32_t>(is, msg.crc_);
  read<uint32_t>(is, msg.reserved_);
  read<uint32_t>(is, msg.opcode_);
  read<uint32_t>(is, msg.res_code_);

  if (msg.crc_ != calcCRC(msg))
  {
    throw DecodeCRCMismatchException();
  }

  return msg;
}

template <typename T>
inline void ReplyMsgFromScanner::read(std::istringstream& is, const T& data)
{
  // Alternatives for transformation:
  // typedef boost::iostreams::basic_array_source<char> Device;
  // boost::iostreams::stream<Device> stream((char*)&data, sizeof(DataReply::MemoryFormat));

  is.read((char*)&data, sizeof(T));
  return;
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

  write<uint32_t>(os, crc);
  write<uint32_t>(os, reserved_);
  write<uint32_t>(os, opcode_);
  write<uint32_t>(os, res_code_);

  // TODO check limits
  std::string data_str(os.str());
  assert(data_str.length() == REPLY_MSG_FROM_SCANNER_SIZE && "Message data of start reply has not the expected size");

  ReplyMsgFromScanner::RawType ret_val{};
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

template <typename T>
inline void ReplyMsgFromScanner::write(std::ostringstream& os, const T& data) const
{
  os.write((char*)&data, sizeof(T));
  return;
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_REPLY_MSG_FROM_SCANNER_H
