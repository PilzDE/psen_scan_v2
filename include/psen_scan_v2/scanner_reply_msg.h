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

#ifndef PSEN_SCAN_V2_SCANNER_REPLY_MSG_H
#define PSEN_SCAN_V2_SCANNER_REPLY_MSG_H

#include <cstdint>
#include <cassert>
#include <array>
#include <sstream>

#include <boost/crc.hpp>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/crc_mismatch_exception.h"

namespace psen_scan_v2
{
/**
 * @brief Defines the possible types of reply messages which can be received from the scanner.
 */
enum class ScannerReplyMsgType
{
  Start,
  Stop,
  Unknown
};

static constexpr std::size_t REPLY_MSG_FROM_SCANNER_SIZE = 16;  // See protocol description

/**
 * @brief Higher level data type representing a reply message from the scanner.
 */
class ScannerReplyMsg
{
public:
  //! @brief Deserializes the specified data into a reply message.
  static ScannerReplyMsg fromRawData(const MaxSizeRawData& data);

public:
  /**
   * @brief Constructor.
   *
   * @param op_code The operation code contained in the raw data of the reply message.
   * @param res_code The result code contained in the raw data of the reply message.
   * - If the message is accepted, the returned value is 0x00.
   * - If the message is refused, the returned value is 0xEB.
   * - If the CRC is not correct, the device will not send any message.
   */
  ScannerReplyMsg(const uint32_t op_code, const uint32_t res_code);

public:
  ScannerReplyMsgType type() const;

public:
  static uint32_t getStartOpCode();
  static uint32_t getStopOpCode();
  static uint32_t calcCRC(const ScannerReplyMsg& msg);

  using RawType = FixedSizeRawData<REPLY_MSG_FROM_SCANNER_SIZE>;
  //! @brief Serializes the reply into raw data.
  RawType toRawData() const;

private:
  ScannerReplyMsg() = delete;

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
  static constexpr uint32_t OPCODE_STOP{ 0x36 };
};

inline uint32_t ScannerReplyMsg::calcCRC(const ScannerReplyMsg& msg)
{
  boost::crc_32_type result;
  // Read all data except the field of the sent crc at the beginning according to:
  // Reference Guide Rev. A – November 2019 Page 14
  result.process_bytes(&(msg.reserved_), sizeof(ScannerReplyMsg::reserved_));
  result.process_bytes(&(msg.opcode_), sizeof(ScannerReplyMsg::opcode_));
  result.process_bytes(&(msg.res_code_), sizeof(ScannerReplyMsg::res_code_));
  return result.checksum();
}

inline uint32_t ScannerReplyMsg::getStartOpCode()
{
  return OPCODE_START;
}

inline uint32_t ScannerReplyMsg::getStopOpCode()
{
  return OPCODE_STOP;
}

inline ScannerReplyMsg::ScannerReplyMsg(const uint32_t op_code, const uint32_t res_code)
  : opcode_(op_code), res_code_(res_code)
{
  crc_ = calcCRC(*this);
}

inline ScannerReplyMsg ScannerReplyMsg::fromRawData(const MaxSizeRawData& data)
{
  ScannerReplyMsg msg{ 0, 0 };

  // Alternatives for transformation:
  // typedef boost::iostreams::basic_array_source<char> Device;
  // boost::iostreams::stream<Device> stream((char*)&data, sizeof(DataReply::MemoryFormat));

  std::istringstream stream(std::string((char*)&data, REPLY_MSG_FROM_SCANNER_SIZE));

  stream.read((char*)&msg.crc_, sizeof(ScannerReplyMsg::crc_));
  stream.read((char*)&msg.reserved_, sizeof(ScannerReplyMsg::reserved_));
  stream.read((char*)&msg.opcode_, sizeof(ScannerReplyMsg::opcode_));
  stream.read((char*)&msg.res_code_, sizeof(ScannerReplyMsg::res_code_));

  if (msg.crc_ != calcCRC(msg))
  {
    throw CRCMismatch();
  }

  return msg;
}

inline ScannerReplyMsgType ScannerReplyMsg::type() const
{
  if (opcode_ == OPCODE_START)
  {
    return ScannerReplyMsgType::Start;
  }
  else if (opcode_ == OPCODE_STOP)
  {
    return ScannerReplyMsgType::Stop;
  }
  return ScannerReplyMsgType::Unknown;
}

inline ScannerReplyMsg::RawType ScannerReplyMsg::toRawData() const
{
  std::ostringstream os;

  uint32_t crc{ calcCRC(*this) };
  os.write((char*)&crc, sizeof(uint32_t));
  os.write((char*)&reserved_, sizeof(uint32_t));
  os.write((char*)&opcode_, sizeof(uint32_t));
  os.write((char*)&res_code_, sizeof(uint32_t));

  ScannerReplyMsg::RawType ret_val{};

  // TODO check limits
  std::string data_str(os.str());
  assert(data_str.length() == REPLY_MSG_FROM_SCANNER_SIZE && "Message data of start reply has not the expected size");
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_REPLY_MSG_H
