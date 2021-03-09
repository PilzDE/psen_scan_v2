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

#ifndef PSEN_SCAN_V2_STANDALONE_TESTS_SCANNER_MOCK_H
#define PSEN_SCAN_V2_STANDALONE_TESTS_SCANNER_MOCK_H

#include <functional>
#include <iostream>
#include <string>

#include <boost/asio.hpp>

#include <gmock/gmock.h>

// Test framework
#include "psen_scan_v2_standalone/communication_layer/mock_udp_server.h"

// Software under testing
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"

namespace psen_scan_v2_standalone_test
{
struct PortHolder
{
  PortHolder& operator++();

  void printPorts() const;

  const int MIN_DATA_PORT_HOST{ 45000 };
  const int MAX_DATA_PORT_HOST{ 46000 };

  const int MIN_CONTROL_PORT_HOST{ 57000 };
  const int MAX_CONTROL_PORT_HOST{ 58000 };

  const unsigned short MIN_CONTROL_PORT_SCANNER{ 3000u };
  const unsigned short MAX_CONTROL_PORT_SCANNER{ 4000u };

  const unsigned short MIN_DATA_PORT_SCANNER{ 7000u };
  const unsigned short MAX_DATA_PORT_SCANNER{ 8000u };

  int data_port_host{ MIN_DATA_PORT_HOST };
  int control_port_host{ MIN_CONTROL_PORT_HOST };

  unsigned short control_port_scanner{ MIN_CONTROL_PORT_SCANNER };
  unsigned short data_port_scanner{ MIN_DATA_PORT_SCANNER };
};

static PortHolder GLOBAL_PORT_HOLDER;

using boost::asio::ip::udp;
using namespace psen_scan_v2_standalone;

class ScannerMock
{
public:
  ScannerMock(const std::string& host_ip, const PortHolder& port_holder);

public:
  MOCK_METHOD2(receiveControlMsg,
               void(const udp::endpoint&, const psen_scan_v2_standalone::data_conversion_layer::RawData&));
  MOCK_METHOD2(receiveDataMsg,
               void(const udp::endpoint&, const psen_scan_v2_standalone::data_conversion_layer::RawData&));

public:
  void startListeningForControlMsg();
  void startContinuousListeningForControlMsg();

public:
  void sendStartReply();
  void sendStopReply();
  void sendMonitoringFrame(const data_conversion_layer::monitoring_frame::Message& msg);
  void sendEmptyMonitoringFrame();

private:
  void sendReply(const data_conversion_layer::scanner_reply::Message::Type& reply_type);

private:
  const udp::endpoint control_msg_receiver_;
  const udp::endpoint monitoring_frame_receiver_;

  MockUDPServer control_server_;
  MockUDPServer data_server_;
};

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_SCANNER_MOCK_H
