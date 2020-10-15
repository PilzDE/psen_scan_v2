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
#include <mutex>

#include <gtest/gtest_prod.h>

#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/function_pointers.h"
#include "psen_scan_v2/laserscan_conversions.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/stop_request.h"
#include "psen_scan_v2/udp_client.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/watchdog.h"
#include "psen_scan_v2/logging.h"

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
  ScannerControllerT(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback);
  std::future<void> start();
  std::future<void> stop();

private:
  void handleError(const std::string& error_msg);
  void handleMonitoringFrame(const MonitoringFrameMsg& frame);
  void handleScannerReply(const MaxSizeRawData& data, const std::size_t& num_bytes);
  void handleUdpData(const MaxSizeRawData& data, const std::size_t& num_bytes);

  void sendStartRequest();
  void sendStopRequest();

private:
  void handleStartReplyTimeout();
  void handleStopReplyTimeout(const std::string& error_str);

  void notifyStartedState();
  void notifyStoppedState();

  void startStartReplyWatchdog();
  void stopStartReplyWatchdog();

private:
  ScannerConfiguration scanner_config_;
  TCSM state_machine_;

  TUCI control_udp_client_;
  TUCI data_udp_client_;

  // The watchdog pointer is changed by the user-main-thread and by the UDP client callback-thread/io-service-thread,
  // and, therefore, needs to be protected/sychronized via mutex.
  std::mutex watchdog_mutex_;
  std::unique_ptr<Watchdog> start_reply_watchdog_{};

  LaserScanCallback laser_scan_callback_;

  std::promise<void> started_;
  std::promise<void> stopped_;

  friend class ScannerControllerTest;
  FRIEND_TEST(ScannerControllerTest, testSuccessfulStartSequence);
  FRIEND_TEST(ScannerControllerTest, testSuccessfulStopSequence);
  FRIEND_TEST(ScannerControllerTest, testHandleNewMonitoringFrame);
  FRIEND_TEST(ScannerControllerTest, testHandleEmptyMonitoringFrame);
  FRIEND_TEST(ScannerControllerTest, testHandleError);
  FRIEND_TEST(ScannerControllerTest, testRetryAfterStartReplyTimeout);
  FRIEND_TEST(ScannerControllerTest, testStopReplyTimeout);
  FRIEND_TEST(ScannerControllerTest, testStopWhileWaitingForStartReply);
};

typedef ScannerControllerT<> ScannerController;

template <typename TCSM, typename TUCI>
ScannerControllerT<TCSM, TUCI>::ScannerControllerT(const ScannerConfiguration& scanner_config,
                                                   const LaserScanCallback& laser_scan_callback)
  : scanner_config_(scanner_config)
  , state_machine_(std::bind(&ScannerControllerT::handleMonitoringFrame, this, std::placeholders::_1),
                   std::bind(&ScannerControllerT::sendStartRequest, this),
                   std::bind(&ScannerControllerT::sendStopRequest, this),
                   std::bind(&ScannerControllerT::notifyStartedState, this),
                   std::bind(&ScannerControllerT::notifyStoppedState, this))
  , control_udp_client_(
        std::bind(&ScannerControllerT::handleScannerReply, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
        scanner_config.hostUDPPortControl(),
        scanner_config.clientIp(),
        CONTROL_PORT_OF_SCANNER_DEVICE)
  , data_udp_client_(std::bind(&ScannerControllerT::handleUdpData, this, std::placeholders::_1, std::placeholders::_2),
                     std::bind(&ScannerControllerT::handleError, this, std::placeholders::_1),
                     scanner_config.hostUDPPortData(),
                     scanner_config.clientIp(),
                     DATA_PORT_OF_SCANNER_DEVICE)
  , laser_scan_callback_(laser_scan_callback)
{
  if (!laser_scan_callback)
  {
    throw std::invalid_argument("Laserscan callback must not be null");
  }
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleUdpData(const MaxSizeRawData& data, const std::size_t& num_bytes)
{
  MonitoringFrameMsg frame{ deserialize_monitoring_frame(data, num_bytes) };
  state_machine_.processMonitoringFrameReceivedEvent(frame);
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleMonitoringFrame(const MonitoringFrameMsg& frame)
{
  std::ostringstream os;
  for (const auto& message : frame.diagnosticMessages())
  {
    os << message;
  }
  PSENSCAN_WARN_THROTTLE(1, "ScannerController", "{}", os.str());

  if (frame.measures().empty())
  {
    return;
  }

  LaserScan scan{ toLaserScan(frame) };
  laser_scan_callback_(scan);
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleError(const std::string& error_msg)
{
  PSENSCAN_ERROR("ScannerController", error_msg);
  // TODO: Add implementation -> Tell state machine about error
}

template <typename TCSM, typename TUCI>
std::future<void> ScannerControllerT<TCSM, TUCI>::start()
{
  control_udp_client_.startAsyncReceiving(ReceiveMode::single);
  data_udp_client_.startAsyncReceiving();

  state_machine_.processStartRequestEvent();
  return started_.get_future();
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
  startStartReplyWatchdog();
  control_udp_client_.write(StartRequest(scanner_config_, DEFAULT_SEQ_NUMBER).serialize());
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleScannerReply(const MaxSizeRawData& data, const std::size_t& num_bytes)
{
  ScannerReplyMsg frame{ ScannerReplyMsg::deserialize(data) };
  state_machine_.processReplyReceivedEvent(frame.type());
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleStartReplyTimeout()
{
  PSENSCAN_ERROR("ScannerController",
                 "Timeout while waiting for the scanner to start! Retrying... "
                 "(Please check the ethernet connection or contact PILZ support if the error persists.)");
  state_machine_.processStartReplyTimeoutEvent();
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::handleStopReplyTimeout(const std::string& error_str)
{
  PSENSCAN_ERROR(
      "ScannerController", "Timeout while waiting for stop reply message from scanner | Error message: {}", error_str);
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::sendStopRequest()
{
  // Before we send the stop request, we need to ensure that an potentially running start()-loop is stopped.
  stopStartReplyWatchdog();

  control_udp_client_.startAsyncReceiving(
      ReceiveMode::single, std::bind(&ScannerControllerT::handleStopReplyTimeout, this, _1), RECEIVE_TIMEOUT_CONTROL);
  StopRequest stop_request;
  control_udp_client_.write(stop_request.serialize());
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::notifyStartedState()
{
  PSENSCAN_DEBUG("ScannerController", "Started() called.");
  stopStartReplyWatchdog();

  started_.set_value();
  // Reinitialize
  started_ = std::promise<void>();
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::notifyStoppedState()
{
  PSENSCAN_DEBUG("ScannerController", "Stopped() called.");
  stopped_.set_value();

  // Reinitialize
  stopped_ = std::promise<void>();
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::startStartReplyWatchdog()
{
  // If the watchdog exists, the watchdog is running and there is nothing to do.
  if (start_reply_watchdog_)
  {
    return;
  }

  const std::lock_guard<std::mutex> lock(watchdog_mutex_);
  start_reply_watchdog_ = std::make_unique<Watchdog>(RECEIVE_TIMEOUT_CONTROL,
                                                     std::bind(&ScannerControllerT::handleStartReplyTimeout, this));
}

template <typename TCSM, typename TUCI>
void ScannerControllerT<TCSM, TUCI>::stopStartReplyWatchdog()
{
  const std::lock_guard<std::mutex> lock(watchdog_mutex_);
  start_reply_watchdog_.reset(nullptr);
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONTROLLER_H
