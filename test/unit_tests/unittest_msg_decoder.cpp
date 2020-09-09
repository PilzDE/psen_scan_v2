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

#include <cstring>
#include <string>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/reply_msg_from_scanner.h"
#include "psen_scan_v2/msg_decoder.h"

using namespace psen_scan_v2;

static constexpr uint32_t DEFAULT_RESULT_CODE{ 0 };

class MockCallbackHolder
{
public:
  MOCK_METHOD0(start_reply_callback, void());
  MOCK_METHOD1(error_callback, void(const std::string&));
};

/**
 * Testing if a StartReply message can be identified correctly with the correct crc value.
 * This should call the start_reply_callback method.
 */
TEST(MsgDecoderTest, decodeStartReply)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::error_callback, &mock, std::placeholders::_1));

  ReplyMsgFromScanner reply{ ReplyMsgFromScanner::getStartOpCode(), DEFAULT_RESULT_CODE };

  ReplyMsgFromScanner::RawType reply_raw{ reply.toCharArray() };
  RawScannerData raw_data{};
  std::copy(reply_raw.begin(), reply_raw.end(), raw_data.begin());

  EXPECT_CALL(mock, start_reply_callback()).Times(1);

  decoder.decodeAndDispatch(raw_data, REPLY_MSG_FROM_SCANNER_SIZE);
}

TEST(MsgDecoderTest, decodeStartReplyCrcFail)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::error_callback, &mock, std::placeholders::_1));

  ReplyMsgFromScanner reply{ ReplyMsgFromScanner::getStartOpCode(), DEFAULT_RESULT_CODE };

  ReplyMsgFromScanner::RawType reply_raw{ reply.toCharArray() };
  RawScannerData raw_data{};
  std::copy(reply_raw.begin(), reply_raw.end(), raw_data.begin());
  raw_data[0] = 'a';

  EXPECT_CALL(mock, start_reply_callback()).Times(0);

  EXPECT_THROW(decoder.decodeAndDispatch(raw_data, REPLY_MSG_FROM_SCANNER_SIZE), CRCMismatch);
}

/**
 * Testing if a StartReply message can not be identified correctly with incorrect size.
 * This should *not* call the start_reply_callback method and raise a NotImplementedException.
 */
TEST(MsgDecoderTest, decodeStartReplyWrongSizeNotImplemented)
{
  MockCallbackHolder mock;
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::error_callback, &mock, std::placeholders::_1));

  ReplyMsgFromScanner reply{ ReplyMsgFromScanner::getStartOpCode(), DEFAULT_RESULT_CODE };

  ReplyMsgFromScanner::RawType reply_raw{ reply.toCharArray() };
  RawScannerData raw_data{};
  std::copy(reply_raw.begin(), reply_raw.end(), raw_data.begin());

  EXPECT_CALL(mock, start_reply_callback()).Times(0);
  EXPECT_CALL(mock, error_callback(::testing::_)).Times(1);

  decoder.decodeAndDispatch(raw_data, REPLY_MSG_FROM_SCANNER_SIZE + 1);
}

/**
 * Testing if a StartReply message can not be identified correctly with incorrect opcode.
 * This should *not* call the start_reply_callback method and raise a NotImplementedException.
 */
TEST(MsgDecoderTest, decodeWrongOpCodeNotImplemented)
{
  MockCallbackHolder mock;  // Needed
  MsgDecoder decoder(std::bind(&MockCallbackHolder::start_reply_callback, &mock),
                     std::bind(&MockCallbackHolder::error_callback, &mock, std::placeholders::_1));

  ReplyMsgFromScanner reply{ ReplyMsgFromScanner::getStartOpCode() + 1, DEFAULT_RESULT_CODE };

  ReplyMsgFromScanner::RawType reply_raw{ reply.toCharArray() };
  RawScannerData raw_data{};
  std::copy(reply_raw.begin(), reply_raw.end(), raw_data.begin());

  EXPECT_CALL(mock, start_reply_callback()).Times(0);
  EXPECT_CALL(mock, error_callback(::testing::_)).Times(1);

  decoder.decodeAndDispatch(raw_data, REPLY_MSG_FROM_SCANNER_SIZE);
}

TEST(MsgDecoderTest, testCRCMismatchForCompleteCoverage)
{
  std::unique_ptr<CRCMismatch> ex{ new CRCMismatch() };
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
