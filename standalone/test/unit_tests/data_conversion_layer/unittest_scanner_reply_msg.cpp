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

#include <cstdlib>
#include <functional>
#include <sstream>

#include <boost/crc.hpp>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_serialization_deserialization.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_test_helper.h"

using namespace psen_scan_v2_standalone;

using Message = data_conversion_layer::scanner_reply::Message;

namespace psen_scan_v2_standalone_test
{
static constexpr uint32_t OP_CODE_START{ 0x35 };

TEST(ScannerReplyMsgTest, testTypeStart)
{
  const Message msg(Message::Type::start, Message::OperationResult::accepted);
  EXPECT_EQ(Message::Type::start, msg.type());
}

TEST(ScannerReplyMsgTest, testTypeUnknown)
{
  const Message msg(Message::Type::unknown, Message::OperationResult::accepted);
  EXPECT_EQ(Message::Type::unknown, msg.type());
}

TEST(ScannerReplyMsgTest, testGetStartOpCode)
{
  EXPECT_EQ(OP_CODE_START, static_cast<uint32_t>(Message::Type::start));
}

TEST(ScannerReplyMsgTest, testserialize)
{
  const Message msg(Message::Type::start, Message::OperationResult::accepted);
  const auto raw_msg{ data_conversion_layer::scanner_reply::serialize(msg) };

  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg.at(sizeof(uint32_t)), raw_msg.size() - sizeof(uint32_t));

  const uint32_t reserved{ 0 };

  EXPECT_TRUE(DecodingEquals(raw_msg, 0x00, crc.checksum()));
  EXPECT_TRUE(DecodingEquals(raw_msg, 0x04, reserved));
  EXPECT_TRUE(DecodingEquals(raw_msg, 0x08, static_cast<uint32_t>(Message::Type::start)));
  EXPECT_TRUE(DecodingEquals(raw_msg, 0x0C, static_cast<uint32_t>(Message::OperationResult::accepted)));
}

TEST(ScannerReplyMsgTest, testdeserializeUnknownFields)
{
  const Message msg(Message::Type::unknown, Message::OperationResult::unknown);
  const auto raw_msg{ data_conversion_layer::scanner_reply::serialize(msg) };
  const auto deserialized{ data_conversion_layer::scanner_reply::deserialize(raw_msg) };

  EXPECT_EQ(deserialized.type(), Message::Type::unknown);
  EXPECT_EQ(deserialized.result(), Message::OperationResult::unknown);
}

TEST(ScannerReplyMsgTest, testdeserializeValidCRC)
{
  // Use raw data generated from serialize()
  const Message msg(Message::Type::start, Message::OperationResult::accepted);
  const auto raw_msg{ data_conversion_layer::scanner_reply::serialize(msg) };

  const data_conversion_layer::RawData data(raw_msg.begin(), raw_msg.end());

  // Check equality by comparing crc checksums
  boost::crc_32_type crc;
  crc.process_bytes(&raw_msg[sizeof(uint32_t)], raw_msg.size() - sizeof(uint32_t));
  EXPECT_TRUE(DecodingEquals(raw_msg, 0x00, crc.checksum()));
}

TEST(ScannerReplyMsgTest, testdeserializeInvalidCRC)
{
  // Use raw data generated from serialize()
  const Message msg(Message::Type::start, Message::OperationResult::accepted);
  auto raw_msg{ data_conversion_layer::scanner_reply::serialize(msg) };
  raw_msg.at(0) += 0x01;  // alter crc checksum

  const data_conversion_layer::RawData data(raw_msg.begin(), raw_msg.end());
  EXPECT_THROW(data_conversion_layer::scanner_reply::deserialize(data),
               data_conversion_layer::scanner_reply::CRCMismatch);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
