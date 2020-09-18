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

#ifndef PSEN_SCAN_V2_SCANNER_CONTROLLER_H
#define PSEN_SCAN_V2_SCANNER_CONTROLLER_H

#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <gtest/gtest_prod.h>

#include "psen_scan_v2/msg_decoder.h"
#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/udp_client.h"
#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/laserscan_builder.h"

namespace psen_scan_v2
{
// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };

static constexpr std::chrono::milliseconds RECEIVE_TIMEOUT{ 1000 };

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0 };

template <typename TCSM = ControllerStateMachine, typename TUCI = UdpClientImpl>
class ScannerControllerT
{
public:
  ScannerControllerT(const ScannerConfiguration& scanner_config);
  void start();
  void stop();

  void handleError(const std::string& error_msg);
  void sendStartRequest();
  LaserScan buildLaserScan();
  void handleNewData(const RawScannerData& data, const std::size_t& bytes_received);

private:
  ScannerConfiguration scanner_config_;
  TCSM state_machine_;
  MsgDecoder control_msg_decoder_;
  TUCI control_udp_client_;
  TUCI data_udp_client_;
  LaserScanBuilder laser_scan_builder_;

  friend class ScannerControllerTest;
  FRIEND_TEST(ScannerControllerTest, test_start_method_calls_correct_state_machine_event);
  FRIEND_TEST(ScannerControllerTest, test_stop_method_calls_correct_state_machine_event);
  FRIEND_TEST(ScannerControllerTest, test_udp_clients_listen_before_sending_start_request);
};

typedef ScannerControllerT<> ScannerController;

template <typename TCSM, typename TUCI>
ScannerControllerT<TCSM, TUCI>::ScannerControllerT(const ScannerConfiguration& scanner_config)
  : scanner_config_(scanner_config)
  , state_machine_(std::bind(&ScannerControllerT::sendStartRequest, this))
  , control_msg_decoder_(std::bind(&TCSM::processStartReplyReceivedEvent, &state_machine_),
                         std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
                         std::bind(&LaserScanBuilder::add, &laser_scan_builder_, std::placeholders::_1))
  , control_udp_client_(
        std::bind(&MsgDecoder::decodeAndDispatch, &control_msg_decoder_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
        scanner_config.hostUDPPortControl(),
        scanner_config.clientIp(),
        CONTROL_PORT_OF_SCANNER_DEVICE)
  , data_udp_client_(
        std::bind(&ScannerControllerT::handleNewData, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
        scanner_config.hostUDPPortData(),
        scanner_config.clientIp(),
        DATA_PORT_OF_SCANNER_DEVICE)
{
}


template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleNewData(const RawScannerData& data, const std::size_t& bytes_received)
{
  MonitoringFrameMsg frame{ MonitoringFrameMsg::fromRawData(data) };
  laser_scan_builder_.add(frame);
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleError(const std::string& error_msg)
{
  PSENSCAN_ERROR("ScannerController", error_msg);
  // TODO: Add implementation -> Tell state machine about error
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::start()
{
  state_machine_.processStartRequestEvent();
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::stop()
{
  // TODO: Impl. sending of StopRequest
  state_machine_.processStopRequestEvent();
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::sendStartRequest()
{
  control_udp_client_.startReceiving(RECEIVE_TIMEOUT);
  data_udp_client_.startReceiving(RECEIVE_TIMEOUT);
  StartRequest start_request(scanner_config_, DEFAULT_SEQ_NUMBER);

  control_udp_client_.write(start_request.toRawType());
}

template <typename TCSM, typename TUCI>
LaserScan ScannerControllerT<TCSM, TUCI>::buildLaserScan()
{
  constexpr std::chrono::milliseconds sleeping_time{ 10 };

  using cl = std::chrono::system_clock;
  cl::time_point start_time = cl::now();

  int i=0;
  while (!laser_scan_builder_.laserScanReady())
  {
    std::this_thread::sleep_for(sleeping_time);
    if ((0 == i++ % 100) && (cl::now() - start_time > std::chrono::seconds(5)))
    {
      throw LaserScanBuildFailure("Timeout reached, while waiting for laserscan data.");
    }
  }

  return laser_scan_builder_.build();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONTROLLER_H
