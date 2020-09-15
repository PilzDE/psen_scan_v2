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
  MOCK_METHOD0(stop_reply_callback, void());
  MOCK_METHOD1(error_callback, void(const std::string&));
};

static MaxSizeRawData buildRawData(const uint32_t op_code, const uint32_t result_code)
{
  MaxSizeRawData raw_data;
  ScannerReplyMsg::RawType reply_raw(ScannerReplyMsg(op_code, result_code).toRawData());
  std::copy(reply_raw.begin(), reply_raw.end(), raw_data.begin());
  return raw_data;
}

class MsgDecoderTest : public ::testing::Test
{
protected:
  MockCallbackHolder mock_;

  MsgDecoder decoder_{ std::bind(&MockCallbackHolder::start_reply_callback, &mock_),
                       std::bind(&MockCallbackHolder::stop_reply_callback, &mock_),
                       std::bind(&MockCallbackHolder::error_callback, &mock_, std::placeholders::_1) };
};

TEST_F(MsgDecoderTest, testCorrectStartReply)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(1);
  EXPECT_CALL(mock_, stop_reply_callback()).Times(0);

  const auto raw_start_reply{ buildRawData(ScannerReplyMsg::getStartOpCode(), DEFAULT_RESULT_CODE) };
  decoder_.decodeAndDispatch(raw_start_reply, REPLY_MSG_FROM_SCANNER_SIZE);
}

TEST_F(MsgDecoderTest, testCorrectStopReply)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(0);
  EXPECT_CALL(mock_, stop_reply_callback()).Times(1);

  const auto raw_stop_reply{ buildRawData(ScannerReplyMsg::getStopOpCode(), DEFAULT_RESULT_CODE) };
  decoder_.decodeAndDispatch(raw_stop_reply, REPLY_MSG_FROM_SCANNER_SIZE);
}

TEST_F(MsgDecoderTest, testIncorrectCrCForStartReply)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(0);

  auto raw_reply{ buildRawData(ScannerReplyMsg::getStartOpCode(), DEFAULT_RESULT_CODE) };
  raw_reply[0] = 'a';
  EXPECT_THROW(decoder_.decodeAndDispatch(raw_reply, REPLY_MSG_FROM_SCANNER_SIZE), CRCMismatch);
}

TEST_F(MsgDecoderTest, testIncorrectCrCForStopReply)
{
  EXPECT_CALL(mock_, stop_reply_callback()).Times(0);

  auto raw_reply{ buildRawData(ScannerReplyMsg::getStopOpCode(), DEFAULT_RESULT_CODE) };
  raw_reply[0] = 'a';
  EXPECT_THROW(decoder_.decodeAndDispatch(raw_reply, REPLY_MSG_FROM_SCANNER_SIZE), CRCMismatch);
}

TEST_F(MsgDecoderTest, testIncorrectReplySize)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(0);
  EXPECT_CALL(mock_, error_callback(::testing::_)).Times(1);

  const auto raw_reply{ buildRawData(ScannerReplyMsg::getStartOpCode(), DEFAULT_RESULT_CODE) };
  decoder_.decodeAndDispatch(raw_reply, REPLY_MSG_FROM_SCANNER_SIZE + 1);
}

TEST_F(MsgDecoderTest, testIncorrectOPCode)
{
  EXPECT_CALL(mock_, start_reply_callback()).Times(0);
  EXPECT_CALL(mock_, error_callback(::testing::_)).Times(1);

  const auto raw_wrong_reply{ buildRawData(ScannerReplyMsg::getStartOpCode() + 10, DEFAULT_RESULT_CODE) };
  decoder_.decodeAndDispatch(raw_wrong_reply, REPLY_MSG_FROM_SCANNER_SIZE);
}

TEST_F(MsgDecoderTest, testCRCMismatchForCompleteCoverage)
{
  std::unique_ptr<CRCMismatch> ex{ new CRCMismatch() };
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
