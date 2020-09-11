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
  static ScannerReplyMsg fromRawData(const RawScannerData& data);

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
  static uint32_t calcCRC(const ScannerReplyMsg& msg);

private:
  template <typename T>
  static void process_bytes(boost::crc_32_type& crc_32, const T& data);

public:
  using RawType = std::array<char, REPLY_MSG_FROM_SCANNER_SIZE>;
  RawType toCharArray();

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
};

inline uint32_t ScannerReplyMsg::calcCRC(const ScannerReplyMsg& msg)
{
  boost::crc_32_type result;
  // Read all data except the field of the sent crc at the beginning according to:
  // Reference Guide Rev. A – November 2019 Page 14
  processBytes(result, msg.reserved_);
  processBytes(result, msg.opcode_);
  processBytes(result, msg.res_code_);
  return result.checksum();
}

template <typename T>
inline void ReplyMsgFromScanner::processBytes(boost::crc_32_type& crc_32, const T& data)
{
  crc_32.process_bytes(&data, sizeof(T));
  return;
}

inline uint32_t ReplyMsgFromScanner::getStartOpCode()
{
  return OPCODE_START;
}

inline ScannerReplyMsg::ScannerReplyMsg(const uint32_t op_code, const uint32_t res_code)
  : opcode_(op_code), res_code_(res_code)
{
  crc_ = calcCRC(*this);
}

inline ScannerReplyMsg ScannerReplyMsg::fromRawData(const RawScannerData& data)
{
  ScannerReplyMsg msg{ 0, 0 };

  RawScannerData tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), REPLY_MSG_FROM_SCANNER_SIZE));

  read(is, msg.crc_);
  read(is, msg.reserved_);
  read(is, msg.opcode_);
  read(is, msg.res_code_);
  
  if (msg.crc_ != calcCRC(msg))
  {
    throw CRCMismatch();
  }

  return msg;
}

template <typename T>
inline void ReplyMsgFromScanner::read(std::istringstream& is, T& data)
{
  // Alternatives for transformation:
  // typedef boost::iostreams::basic_array_source<char> Device;
  // boost::iostreams::stream<Device> stream((char*)&data, sizeof(DataReply::MemoryFormat));

  is.read(reinterpret_cast<char*>(&data), sizeof(T));
  return;
}

inline ScannerReplyMsgType ScannerReplyMsg::type() const
{
  if (opcode_ == OPCODE_START)
  {
    return ScannerReplyMsgType::Start;
  }
  return ScannerReplyMsgType::Unknown;
}

inline ScannerReplyMsg::RawType ScannerReplyMsg::toCharArray()
{
  std::ostringstream os;

  uint32_t crc{ calcCRC(*this) };

  write(os, crc);
  write(os, reserved_);
  write(os, opcode_);
  write(os, res_code_);

  // TODO check limits
  std::string data_str(os.str());
  assert(data_str.length() == REPLY_MSG_FROM_SCANNER_SIZE && "Message data of start reply has not the expected size");

  ScannerReplyMsg::RawType ret_val{};
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

template <typename T>
inline void ReplyMsgFromScanner::write(std::ostringstream& os, const T& data) const
{
  os.write((char*)(&data), sizeof(T));
  return;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_REPLY_MSG_H
