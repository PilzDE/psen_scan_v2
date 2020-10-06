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

#ifndef PSEN_SCAN_V2_TEST_MOCK_SCANNER_IMPL_H
#define PSEN_SCAN_V2_TEST_MOCK_SCANNER_IMPL_H

#include <gmock/gmock.h>
#include <functional>
#include <cstdint>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/ip/address_v4.hpp>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/mock_udp_server.h"
#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"

namespace psen_scan_v2_test
{
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };

static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr uint32_t HOST_UDP_PORT_DATA{ 45000 };
static constexpr uint32_t HOST_UDP_PORT_CONTROL{ 57000 };

using std::placeholders::_1;
using std::placeholders::_2;

class ScannerMock
{
public:
  MOCK_METHOD2(receiveControlMsg, void(const udp::endpoint&, const psen_scan_v2::DynamicSizeRawData&));
  MOCK_METHOD2(receiveDataMsg, void(const udp::endpoint&, const psen_scan_v2::DynamicSizeRawData&));

public:
  void startListeningForControlMsg();

public:
  void sendStartReply();
  void sendStopReply();
  void sendMonitoringFrame();

private:
  void sendReply(const uint32_t reply_type);

private:
  const udp::endpoint control_msg_receiver_{ udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS),
                                                           HOST_UDP_PORT_CONTROL) };

  const udp::endpoint monitoring_frame_receiver_{
    udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), HOST_UDP_PORT_DATA)
  };

  MockUDPServer control_server_{ CONTROL_PORT_OF_SCANNER_DEVICE,
                                 std::bind(&ScannerMock::receiveControlMsg, this, _1, _2) };
  MockUDPServer data_server_{ DATA_PORT_OF_SCANNER_DEVICE, std::bind(&ScannerMock::receiveDataMsg, this, _1, _2) };
};

void ScannerMock::startListeningForControlMsg()
{
  control_server_.asyncReceive();
}

void ScannerMock::sendReply(const uint32_t reply_type)
{
  const psen_scan_v2::ScannerReplyMsg msg(reply_type, 0x00);
  control_server_.asyncSend<psen_scan_v2::REPLY_MSG_FROM_SCANNER_SIZE>(control_msg_receiver_, msg.toRawData());
}

void ScannerMock::sendStartReply()
{
  sendReply(getOpCodeValue(psen_scan_v2::ScannerReplyMsgType::Start));
}

void ScannerMock::sendStopReply()
{
  sendReply(getOpCodeValue(psen_scan_v2::ScannerReplyMsgType::Stop));
}

void ScannerMock::sendMonitoringFrame()
{
  constexpr UDPFrameTestDataWithoutIntensities raw_scan;
  data_server_.asyncSend<raw_scan.hex_dump.size()>(monitoring_frame_receiver_, transformArray(raw_scan.hex_dump));
}

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_MOCK_SCANNER_H
