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

#ifndef PSEN_SCAN_V2_TEST_MOCK_UDP_CLIENT_H
#define PSEN_SCAN_V2_TEST_MOCK_UDP_CLIENT_H

#include <gmock/gmock.h>

#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/udp_client.h"

#include "psen_scan_v2/raw_data_array_conversion.h"

namespace psen_scan_v2_test
{
static constexpr uint32_t OP_CODE_START{ 0x35 };
static constexpr uint32_t OP_CODE_STOP{ 0x36 };
static constexpr uint32_t OP_CODE_UNKNOWN{ 0x01 };
static constexpr uint32_t RES_CODE_ACCEPTED{ 0x00 };

class MockUdpClient
{
public:
  MockUdpClient(const psen_scan_v2::NewDataHandler& data_handler,
                const psen_scan_v2::ErrorHandler& error_handler,
                const unsigned short& host_port,
                const unsigned int& endpoint_ip,
                const unsigned short& endpoint_port)
    : data_handler_(data_handler), error_handler_(error_handler)
  {
    using ::testing::_;
    using ::testing::SaveArg;
    ON_CALL(*this, startAsyncReceiving(_, _, _)).WillByDefault(SaveArg<1>(&timeout_handler_));
  };

public:
  void sendStartReply();
  void sendStopReply();
  template <typename TestData>
  void sendMonitoringFrame(const TestData& test_data);
  void simulateError(const std::string& msg);
  void simulateTimeout(const std::string& msg);

public:
  MOCK_METHOD0(close, void());
  MOCK_METHOD3(startAsyncReceiving,
               void(const psen_scan_v2::ReceiveMode& modi,
                    const psen_scan_v2::TimeoutHandler& timeout_handler,
                    const std::chrono::high_resolution_clock::duration& timeout));
  // "Simulates" function call which uses default values
  MOCK_METHOD0(startAsyncReceiving, void());
  MOCK_METHOD1(write, void(const psen_scan_v2::DynamicSizeRawData& data));

private:
  void handleNewData(const psen_scan_v2::MaxSizeRawData& received_data, const std::size_t& bytes_received);

private:
  psen_scan_v2::NewDataHandler data_handler_;
  psen_scan_v2::ErrorHandler error_handler_;
  psen_scan_v2::TimeoutHandler timeout_handler_;
};

void MockUdpClient::sendStartReply()
{
  const psen_scan_v2::ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);
  const auto data{ msg.toRawData() };
  psen_scan_v2::MaxSizeRawData max_size_data;
  std::copy_n(data.begin(), data.size(), max_size_data.begin());

  handleNewData(max_size_data, max_size_data.size());
}

void MockUdpClient::sendStopReply()
{
  const psen_scan_v2::ScannerReplyMsg msg(OP_CODE_STOP, RES_CODE_ACCEPTED);
  const auto data{ msg.toRawData() };
  psen_scan_v2::MaxSizeRawData max_size_data;
  std::copy_n(data.begin(), data.size(), max_size_data.begin());

  handleNewData(max_size_data, max_size_data.size());
}

template <typename TestData>
void MockUdpClient::sendMonitoringFrame(const TestData& test_data)
{
  const psen_scan_v2::MaxSizeRawData raw_data = convertToMaxSizeRawData(test_data.hex_dump);
  handleNewData(raw_data, raw_data.size());
}

void MockUdpClient::handleNewData(const psen_scan_v2::MaxSizeRawData& received_data, const std::size_t& bytes_received)
{
  data_handler_(received_data, bytes_received);
}

void MockUdpClient::simulateError(const std::string& msg)
{
  error_handler_(msg);
}

void MockUdpClient::simulateTimeout(const std::string& msg)
{
  timeout_handler_(msg);
}

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_MOCK_UDP_CLIENT_H
