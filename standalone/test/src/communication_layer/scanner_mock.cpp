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

#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <boost/asio.hpp>

#include "psen_scan_v2_standalone/communication_layer/mock_udp_server.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_serialization.h"
#include "psen_scan_v2_standalone/communication_layer/scanner_mock.h"

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_serialization_deserialization.h"

using namespace std::chrono_literals;

namespace psen_scan_v2_standalone_test
{
PortHolder& PortHolder::operator++()
{
  data_port_host = (data_port_host + 1) % MAX_DATA_PORT_HOST;
  if (data_port_host == 0)
  {
    data_port_host = MIN_DATA_PORT_HOST;
  }

  control_port_host = (control_port_host + 1) % MAX_CONTROL_PORT_HOST;
  if (control_port_host == 0)
  {
    control_port_host = MIN_CONTROL_PORT_HOST;
  }

  control_port_scanner = (control_port_scanner + 1) % MAX_CONTROL_PORT_SCANNER;
  if (control_port_scanner == 0)
  {
    control_port_scanner = MIN_CONTROL_PORT_SCANNER;
  }

  data_port_scanner = (data_port_scanner + 1) % MAX_DATA_PORT_SCANNER;
  if (data_port_scanner == 0)
  {
    data_port_scanner = MIN_DATA_PORT_SCANNER;
  }

  return *this;
}

void PortHolder::printPorts() const
{
  std::cout << "Host ports:\n"
            << "- data port = " << data_port_host << "\n"
            << "- control port = " << control_port_host << "\n"
            << "Scanner ports:\n"
            << "- data port = " << control_port_scanner << "\n"
            << "- control port = " << data_port_scanner << "\n"
            << std::endl;
}

using std::placeholders::_1;
using std::placeholders::_2;

ScannerMock::ScannerMock(const std::string& host_ip, const PortHolder& port_holder)
  : control_msg_receiver_(
        udp::endpoint(boost::asio::ip::address_v4::from_string(host_ip), port_holder.control_port_host))
  , monitoring_frame_receiver_(
        udp::endpoint(boost::asio::ip::address_v4::from_string(host_ip), port_holder.data_port_host))
  , control_server_(port_holder.control_port_scanner, std::bind(&ScannerMock::receiveControlMsg, this, _1, _2))
  , data_server_(port_holder.data_port_scanner, std::bind(&ScannerMock::receiveDataMsg, this, _1, _2))
{
  startContinuousListeningForControlMsg();
}

void ScannerMock::startContinuousListeningForControlMsg()
{
  control_server_.asyncReceive(MockUDPServer::ReceiveMode::continuous);
}

void ScannerMock::sendReply(const ReplyMsg::Type& reply_type, const ReplyMsg::OperationResult& result)
{
  const ReplyMsg msg(reply_type, result);
  control_server_.asyncSend(control_msg_receiver_, data_conversion_layer::scanner_reply::serialize(msg));
}

void ScannerMock::sendStartReply(const ReplyMsg::OperationResult& result)
{
  std::cout << "ScannerMock: Send start reply..." << std::endl;
  sendReply(ReplyMsg::Type::start, result);
}

void ScannerMock::sendStopReply()
{
  std::cout << "ScannerMock: Send stop reply..." << std::endl;
  sendReply(ReplyMsg::Type::stop);
}

void ScannerMock::sendMonitoringFrame(const MonitoringFrameMsg& msg)
{
  std::cout << "ScannerMock: Send monitoring frame..." << std::endl;
  data_server_.asyncSend(monitoring_frame_receiver_, data_conversion_layer::monitoring_frame::serialize(msg));
}

void ScannerMock::sendEmptyMonitoringFrame()
{
  psen_scan_v2_standalone::data_conversion_layer::RawData data;
  assert(data.empty());
  data_server_.asyncSend(monitoring_frame_receiver_, data);
}

void ScannerMock::sendMonitoringFrames(const std::vector<MonitoringFrameMsg>& msgs)
{
  for (const auto& msg : msgs)
  {
    sendMonitoringFrame(msg);
    // Sleep to ensure that message are not sent too fast which might cause messages overwrite in socket buffer
    std::this_thread::sleep_for(10ms);
  }
}

}  // namespace psen_scan_v2_standalone_test
