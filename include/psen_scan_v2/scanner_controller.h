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

#ifndef PSEN_SCAN_SCANNER_CONTROLLER_H
#define PSEN_SCAN_SCANNER_CONTROLLER_H

#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <gtest/gtest_prod.h>

#include "psen_scan/msg_decoder.h"
#include "psen_scan/controller_state_machine.h"
#include "psen_scan/udp_client.h"
#include "psen_scan/controller_state_machine.h"
#include "psen_scan/start_request.h"
#include "psen_scan/scanner_configuration.h"

namespace psen_scan
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

private:
  ScannerConfiguration scanner_config_;
  TCSM state_machine_;
  MsgDecoder control_msg_decoder_;
  MsgDecoder data_msg_decoder_;
  TUCI control_udp_client_;
  TUCI data_udp_client_;

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
                         std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1))
  , data_msg_decoder_(std::bind(&TCSM::processStartReplyReceivedEvent, &state_machine_),
                      std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1))
  , control_udp_client_(
        std::bind(&MsgDecoder::decodeAndDispatch, &control_msg_decoder_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
        scanner_config.hostUDPPortControl(),
        scanner_config.clientIp(),
        CONTROL_PORT_OF_SCANNER_DEVICE)
  , data_udp_client_(
        std::bind(&MsgDecoder::decodeAndDispatch, &data_msg_decoder_, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
        scanner_config.hostUDPPortData(),
        scanner_config.clientIp(),
        DATA_PORT_OF_SCANNER_DEVICE)
{
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

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_CONTROLLER_H
