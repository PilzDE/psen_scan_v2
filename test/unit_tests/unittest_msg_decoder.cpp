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

#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/msg_decoder.h"

using namespace psen_scan_v2;

static constexpr uint32_t DEFAULT_RESULT_CODE{ 0 };

class MockCallbackHolder
{
public:
  MOCK_METHOD0(start_reply_callback, void());
  MOCK_METHOD1(error_callback, void(const std::string&));
};

class MsgDecoderTest : public ::testing::Test
{
protected:
  MsgDecoderTest()
    : decoder_(std::bind(&MockCallbackHolder::start_reply_callback, &mock_),
               std::bind(&MockCallbackHolder::error_callback, &mock_, std::placeholders::_1))
    , reply_(ReplyMsgFromScanner::getStartOpCode(), DEFAULT_RESULT_CODE)
    , reply_raw_(reply_.toCharArray())
  {
    std::copy(reply_raw_.begin(), reply_raw_.end(), raw_data_.begin());
  }

protected:
  MockCallbackHolder mock_;
  MsgDecoder decoder_;
  ReplyMsgFromScanner reply_;
  ReplyMsgFromScanner::RawType reply_raw_;
  RawScannerData raw_data_;
};

/**
 * Testing if a StartReply message can be identified correctly with the correct crc value.
 * This should call the start_reply_callback method.
 */
TEST_F(MsgDecoderTest, decodeStartReply)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(1);
  decoder_.decodeAndDispatch(raw_data_, REPLY_MSG_FROM_SCANNER_SIZE);
}

TEST_F(MsgDecoderTest, decodeStartReplyCrcFail)
{
  raw_data_[0] = 'a';
  EXPECT_CALL(mock_, start_reply_callback()).Times(0);
  EXPECT_THROW(decoder_.decodeAndDispatch(raw_data_, REPLY_MSG_FROM_SCANNER_SIZE), DecodeCRCMismatchException);
}

/**
 * Testing if a StartReply message can not be identified correctly with incorrect size.
 * This should *not* call the start_reply_callback method and raise a NotImplementedException.
 */
TEST_F(MsgDecoderTest, decodeStartReplyWrongSizeNotImplemented)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(0);
  EXPECT_CALL(mock_, error_callback(::testing::_)).Times(1);
  decoder_.decodeAndDispatch(raw_data_, REPLY_MSG_FROM_SCANNER_SIZE + 1);
}

/**
 * Testing if a StartReply message can not be identified correctly with incorrect opcode.
 * This should *not* call the start_reply_callback method and raise a NotImplementedException.
 */
TEST_F(MsgDecoderTest, decodeWrongOpCodeNotImplemented)
{
  ReplyMsgFromScanner reply{ ReplyMsgFromScanner::getStartOpCode() + 1, DEFAULT_RESULT_CODE };
  ReplyMsgFromScanner::RawType reply_raw{ reply.toCharArray() };
  std::copy(reply_raw.begin(), reply_raw.end(), raw_data_.begin());

  EXPECT_CALL(mock_, start_reply_callback()).Times(0);
  EXPECT_CALL(mock_, error_callback(::testing::_)).Times(1);
  decoder_.decodeAndDispatch(raw_data_, REPLY_MSG_FROM_SCANNER_SIZE);
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
