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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_MOCK_UDP_CLIENT_H
#define PSEN_SCAN_V2_STANDALONE_TEST_MOCK_UDP_CLIENT_H

#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/udp_client.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static constexpr uint32_t OP_CODE_START{ 0x35 };
static constexpr uint32_t OP_CODE_STOP{ 0x36 };
static constexpr uint32_t OP_CODE_UNKNOWN{ 0x01 };
static constexpr uint32_t RES_CODE_ACCEPTED{ 0x00 };

class MockUdpClient
{
public:
  MockUdpClient(const NewDataHandler& data_handler,
                const ErrorHandler& error_handler,
                const unsigned short& host_port,
                const unsigned int& endpoint_ip,
                const unsigned short& endpoint_port)
    : data_handler_(data_handler), error_handler_(error_handler){};

public:
  void sendStartReply();
  void sendStopReply();
  void sendMonitoringFrame(monitoring_frame::Message& msg);
  void simulateError(const std::string& msg);

public:
  MOCK_METHOD0(close, void());
  // "Simulates" function call which uses default values
  MOCK_METHOD0(startAsyncReceiving, void());
  MOCK_METHOD1(startAsyncReceiving, void(const ReceiveMode& modi));
  MOCK_METHOD1(write, void(const data_conversion_layer::RawData& data));

private:
  void handleNewData(const data_conversion_layer::RawData& received_data, const std::size_t& bytes_received);

private:
  NewDataHandler data_handler_;
  ErrorHandler error_handler_;
};

void MockUdpClient::sendStartReply()
{
  const ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);
  const auto data{ msg.serialize() };
  data_conversion_layer::RawData raw_data;
  std::copy_n(data.begin(), data.size(), std::back_inserter(raw_data));

  handleNewData(raw_data, raw_data.size());
}

void MockUdpClient::sendStopReply()
{
  const ScannerReplyMsg msg(OP_CODE_STOP, RES_CODE_ACCEPTED);
  const auto data{ msg.serialize() };
  data_conversion_layer::RawData raw_data;
  std::copy_n(data.begin(), data.size(), std::back_inserter(raw_data));

  handleNewData(raw_data, raw_data.size());
}

void MockUdpClient::sendMonitoringFrame(monitoring_frame::Message& msg)
{
  const data_conversion_layer::RawData msg_raw = convertToRawData(serialize(msg));
  handleNewData(msg_raw, msg_raw.size());
}

void MockUdpClient::handleNewData(const data_conversion_layer::RawData& received_data,
                                  const std::size_t& bytes_received)
{
  data_handler_(received_data, bytes_received);
}

void MockUdpClient::simulateError(const std::string& msg)
{
  error_handler_(msg);
}

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_MOCK_UDP_CLIENT_H
