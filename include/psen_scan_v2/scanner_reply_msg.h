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
#include "psen_scan_v2/raw_processing.h"

namespace psen_scan_v2
{
/**
 * @brief Defines the possible types of reply messages which can be received from the scanner.
 */
enum class ScannerReplyMsgType : uint32_t
{
  Unknown = 0,
  Start = 0x35,
  Stop = 0x36,
};

template <typename TEnum>
auto getOpCodeValue(const TEnum value) -> typename std::underlying_type<TEnum>::type
{
  return static_cast<typename std::underlying_type<TEnum>::type>(value);
}

static constexpr std::size_t REPLY_MSG_FROM_SCANNER_SIZE = 16;  // See protocol description

/**
 * @brief Higher level data type representing a reply message from the scanner.
 */
class ScannerReplyMsg
{
public:
  class CRCMismatch : public std::runtime_error
  {
  public:
    CRCMismatch(const std::string& msg = "CRC did not match!");
  };

public:
  static ScannerReplyMsg deserialize(const MaxSizeRawData& data);

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
  static uint32_t calcCRC(const ScannerReplyMsg& msg);

  using RawType = FixedSizeRawData<REPLY_MSG_FROM_SCANNER_SIZE>;
  RawType serialize() const;

private:
  template <typename T>
  static void processBytes(boost::crc_32_type& crc_32, const T& data);
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
inline void ScannerReplyMsg::processBytes(boost::crc_32_type& crc_32, const T& data)
{
  crc_32.process_bytes(&data, sizeof(T));
}

inline ScannerReplyMsg::ScannerReplyMsg(const uint32_t op_code, const uint32_t res_code)
  : opcode_(op_code), res_code_(res_code)
{
  crc_ = calcCRC(*this);
}

inline ScannerReplyMsg ScannerReplyMsg::deserialize(const MaxSizeRawData& data)
{
  ScannerReplyMsg msg{ 0, 0 };

  MaxSizeRawData tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), REPLY_MSG_FROM_SCANNER_SIZE));

  raw_processing::read(is, msg.crc_);
  raw_processing::read(is, msg.reserved_);
  raw_processing::read(is, msg.opcode_);
  raw_processing::read(is, msg.res_code_);

  if (msg.crc_ != calcCRC(msg))
  {
    throw CRCMismatch();
  }

  return msg;
}

inline ScannerReplyMsgType ScannerReplyMsg::type() const
{
  if (opcode_ == getOpCodeValue(ScannerReplyMsgType::Start))
  {
    return ScannerReplyMsgType::Start;
  }

  if (opcode_ == getOpCodeValue(ScannerReplyMsgType::Stop))
  {
    return ScannerReplyMsgType::Stop;
  }

  return ScannerReplyMsgType::Unknown;
}

inline ScannerReplyMsg::RawType ScannerReplyMsg::serialize() const
{
  std::ostringstream os;

  uint32_t crc{ calcCRC(*this) };

  raw_processing::write(os, crc);
  raw_processing::write(os, reserved_);
  raw_processing::write(os, opcode_);
  raw_processing::write(os, res_code_);

  // TODO check limits
  std::string data_str(os.str());
  assert(data_str.length() == REPLY_MSG_FROM_SCANNER_SIZE && "Message data of start reply has not the expected size");

  ScannerReplyMsg::RawType ret_val{};
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}
inline ScannerReplyMsg::CRCMismatch::CRCMismatch(const std::string& msg) : std::runtime_error(msg)
{
}
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_REPLY_MSG_H
