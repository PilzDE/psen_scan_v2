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
#include <future>
#include <sstream>

#include <gtest/gtest_prod.h>

#include "psen_scan_v2/msg_decoder.h"
#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/udp_client.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/stop_request.h"

namespace psen_scan_v2
{
// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };

static constexpr std::chrono::milliseconds RECEIVE_TIMEOUT_CONTROL{ 1000 };
static constexpr std::chrono::milliseconds RECEIVE_TIMEOUT_DATA{ 1000 };

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0 };

using std::placeholders::_1;

template <typename TCSM = ControllerStateMachine, typename TUCI = UdpClientImpl>
class ScannerControllerT
{
public:
  ScannerControllerT(const ScannerConfiguration& scanner_config);
  void start();
  std::future<void> stop();

  void handleError(const std::string& error_msg);
  void sendStartRequest();
  void sendStopRequest();

private:
  void handleStartReplyTimeout(const std::string& error_str);
  void handleStopReplyTimeout(const std::string& error_str);

private:
  ScannerConfiguration scanner_config_;
  TCSM state_machine_;
  MsgDecoder control_msg_decoder_;
  MsgDecoder data_msg_decoder_;
  TUCI control_udp_client_;
  TUCI data_udp_client_;

  void notifyStoppedState();

  std::promise<void> stopped_;

  friend class ScannerControllerTest;
  FRIEND_TEST(ScannerControllerTest, testStartRequestEvent);
  FRIEND_TEST(ScannerControllerTest, testStartRequestSending);
  FRIEND_TEST(ScannerControllerTest, testStopRequestEvent);
  FRIEND_TEST(ScannerControllerTest, testStopRequestSending);
  FRIEND_TEST(ScannerControllerTest, testHandleStartReplyTimeout);
  FRIEND_TEST(ScannerControllerTest, testStopRequestEventWithFutureUsage);
};

typedef ScannerControllerT<> ScannerController;

template <typename TCSM, typename TUCI>
ScannerControllerT<TCSM, TUCI>::ScannerControllerT(const ScannerConfiguration& scanner_config)
  : scanner_config_(scanner_config)
  , state_machine_(std::bind(&ScannerControllerT::sendStartRequest, this),
                   std::bind(&ScannerControllerT::sendStopRequest, this),
                   std::bind(&ScannerControllerT::notifyStoppedState, this))
  , control_msg_decoder_(std::bind(&TCSM::processStartReplyReceivedEvent, &state_machine_),
                         std::bind(&TCSM::processStopReplyReceivedEvent, &state_machine_),
                         std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1))
  , data_msg_decoder_(std::bind(&TCSM::processStartReplyReceivedEvent, &state_machine_),
                      std::bind(&TCSM::processStopReplyReceivedEvent, &state_machine_),
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
std::future<void> ScannerControllerT<TCSM, TUCI>::stop()
{
  state_machine_.processStopRequestEvent();
  return stopped_.get_future();
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::sendStartRequest()
{
  control_udp_client_.startAsyncReceiving(
      ReceiveMode::single, std::bind(&ScannerControllerT::handleStartReplyTimeout, this, _1), RECEIVE_TIMEOUT_CONTROL);
  data_udp_client_.startAsyncReceiving();
  StartRequest start_request(scanner_config_, DEFAULT_SEQ_NUMBER);

  control_udp_client_.write(start_request.toRawData());
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleStartReplyTimeout(const std::string& error_str)
{
  PSENSCAN_ERROR("ScannerController",
                 "Timeout while waiting for start reply message from scanner | Error message: " << error_str);
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleStopReplyTimeout(const std::string& error_str)
{
  PSENSCAN_ERROR("ScannerController",
                 "Timeout while waiting for stop reply message from scanner | Error message: " << error_str);
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::sendStopRequest()
{
  control_udp_client_.startAsyncReceiving(
      ReceiveMode::single, std::bind(&ScannerControllerT::handleStopReplyTimeout, this, _1), RECEIVE_TIMEOUT_CONTROL);
  StopRequest stop_request;
  control_udp_client_.write(stop_request.toRawData());
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::notifyStoppedState()
{
  PSENSCAN_DEBUG("ScannerController", "Stopped() called.");
  stopped_.set_value();

  // Reinitialize
  stopped_ = std::promise<void>();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONTROLLER_H
