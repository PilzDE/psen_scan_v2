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

#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/raw_data_test_helper.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
static constexpr uint32_t OP_CODE_START{ 0x35 };
static constexpr uint32_t OP_CODE_UNKNOWN{ 0x01 };
static constexpr uint32_t RES_CODE_ACCEPTED{ 0x00 };

TEST(ScannerReplyMsgTest, typeStart)
{
  ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);
  EXPECT_EQ(ScannerReplyMsgType::Start, msg.type());
}

TEST(ScannerReplyMsgTest, typeUnknown)
{
  ScannerReplyMsg msg(OP_CODE_UNKNOWN, RES_CODE_ACCEPTED);
  EXPECT_EQ(ScannerReplyMsgType::Unknown, msg.type());
}

TEST(ScannerReplyMsgTest, getStartOpCode)
{
  EXPECT_EQ(OP_CODE_START, getOpCodeValue(ScannerReplyMsgType::Start));
}

TEST(ScannerReplyMsgTest, testserialize)
{
  const uint32_t op_code{ OP_CODE_START };
  const uint32_t res_code{ RES_CODE_ACCEPTED };

  ScannerReplyMsg msg(op_code, res_code);
  ScannerReplyMsg::RawType raw_msg{ msg.serialize() };

  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));

  const uint32_t reserved{ 0 };

  DecodingEquals(raw_msg, 0x00, crc.checksum());
  DecodingEquals(raw_msg, 0x04, reserved);
  DecodingEquals(raw_msg, 0x08, op_code);
  DecodingEquals(raw_msg, 0x12, res_code);
}

TEST(ScannerReplyMsgTest, calcCRC)
{
  ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);

  // Calculate crc checksum from raw data
  ScannerReplyMsg::RawType raw_msg{ msg.serialize() };
  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));

  EXPECT_EQ(crc.checksum(), ScannerReplyMsg::calcCRC(msg));
}

TEST(ScannerReplyMsgTest, testdeserializeValidCRC)
{
  // Use raw data generated from serialize()
  ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);
  ScannerReplyMsg::RawType raw_msg{ msg.serialize() };

  MaxSizeRawData data;
  std::copy(raw_msg.begin(), raw_msg.end(), data.begin());

  ScannerReplyMsg msg_from_raw{ ScannerReplyMsg::deserialize(data) };

  // Check equality by comparing crc checksums
  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));
  EXPECT_EQ(crc.checksum(), ScannerReplyMsg::calcCRC(msg_from_raw));
}

TEST(ScannerReplyMsgTest, testdeserializeInvalidCRC)
{
  // Use raw data generated from serialize()
  ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);
  ScannerReplyMsg::RawType raw_msg{ msg.serialize() };
  raw_msg[0] += 0x01;  // alter crc checksum

  MaxSizeRawData data;
  std::copy(raw_msg.begin(), raw_msg.end(), data.begin());

  EXPECT_THROW(ScannerReplyMsg::deserialize(data), ScannerReplyMsg::CRCMismatch);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
