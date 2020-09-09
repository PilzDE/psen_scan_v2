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

#include <cstdlib>
#include <functional>
#include <sstream>

#include <boost/crc.hpp>

#include <gtest/gtest.h>

#include "psen_scan_v2/crc_mismatch_exception.h"
#include "psen_scan_v2/reply_msg_from_scanner.h"
#include "psen_scan_v2/raw_data_test_helper.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
static constexpr uint32_t OP_CODE_START{ 0x35 };
static constexpr uint32_t OP_CODE_UNKNOWN{ 0x01 };
static constexpr uint32_t RES_CODE_ACCEPTED{ 0x00 };

TEST(ReplyMsgFromScannerTest, testTypeStart)
{
  ReplyMsgFromScanner msg(OP_CODE_START, RES_CODE_ACCEPTED);
  EXPECT_EQ(ReplyMsgFromScannerType::Start, msg.type());
}

TEST(ReplyMsgFromScannerTest, testTypeUnknown)
{
  ReplyMsgFromScanner msg(OP_CODE_UNKNOWN, RES_CODE_ACCEPTED);
  EXPECT_EQ(ReplyMsgFromScannerType::Unknown, msg.type());
}

TEST(ReplyMsgFromScannerTest, testGetStartOpCode)
{
  EXPECT_EQ(OP_CODE_START, ReplyMsgFromScanner::getStartOpCode());
}

TEST(ReplyMsgFromScannerTest, testToCharArray)
{
  const uint32_t op_code{ OP_CODE_START };
  const uint32_t res_code{ RES_CODE_ACCEPTED };

  ReplyMsgFromScanner msg(op_code, res_code);
  ReplyMsgFromScanner::RawType raw_msg{ msg.toCharArray() };

  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));

  const uint32_t reserved{ 0 };

  DecodingEquals(raw_msg, 0x00, crc.checksum());
  DecodingEquals(raw_msg, 0x04, reserved);
  DecodingEquals(raw_msg, 0x08, op_code);
  DecodingEquals(raw_msg, 0x12, res_code);
}

TEST(ReplyMsgFromScannerTest, testCalcCRC)
{
  ReplyMsgFromScanner msg(OP_CODE_START, RES_CODE_ACCEPTED);

  // Calculate crc checksum from raw data
  ReplyMsgFromScanner::RawType raw_msg{ msg.toCharArray() };
  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));

  EXPECT_EQ(crc.checksum(), ReplyMsgFromScanner::calcCRC(msg));
}

TEST(ReplyMsgFromScannerTest, testFromRawDataValidCRC)
{
  // Use raw data generated from toCharArray()
  ReplyMsgFromScanner msg(OP_CODE_START, RES_CODE_ACCEPTED);
  ReplyMsgFromScanner::RawType raw_msg{ msg.toCharArray() };

  RawScannerData data;
  std::copy(raw_msg.begin(), raw_msg.end(), data.begin());

  ReplyMsgFromScanner msg_from_raw{ ReplyMsgFromScanner::fromRawData(data) };

  // Check equality by comparing crc checksums
  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));
  EXPECT_EQ(crc.checksum(), ReplyMsgFromScanner::calcCRC(msg_from_raw));
}

TEST(ReplyMsgFromScannerTest, testFromRawDataInvalidCRC)
{
  // Use raw data generated from toCharArray()
  ReplyMsgFromScanner msg(OP_CODE_START, RES_CODE_ACCEPTED);
  ReplyMsgFromScanner::RawType raw_msg{ msg.toCharArray() };
  raw_msg[0] += 0x01;  // alter crc checksum

  RawScannerData data;
  std::copy(raw_msg.begin(), raw_msg.end(), data.begin());

  EXPECT_THROW(ReplyMsgFromScanner::fromRawData(data), CRCMismatch);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
