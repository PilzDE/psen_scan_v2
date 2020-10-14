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

#include "psen_scan_v2/scanner_v2.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2::scanner_protocol::scanner_events;

// clang-format off
#define BIND_EVENT(event_name)\
  std::bind(&ScannerV2::triggerEvent<event_name>, this)

#define BIND_RAW_DATA_EVENT(event_name)\
  [this](const MaxSizeRawData& data, const std::size_t& num_bytes){ triggerEventWithParam(event_name(data, num_bytes)); }
// clang-format on

StateMachineArgs* ScannerV2::createStateMachineArgs()
{
  return new StateMachineArgs(IScanner::getConfig(),
                              // UDP clients
                              std::make_unique<UdpClientImpl>(BIND_RAW_DATA_EVENT(RawReplyReceived),
                                                              BIND_EVENT(ReplyReceiveError),
                                                              IScanner::getConfig().hostUDPPortControl(),
                                                              IScanner::getConfig().clientIp(),
                                                              CONTROL_PORT_OF_SCANNER_DEVICE),
                              std::make_unique<UdpClientImpl>(BIND_RAW_DATA_EVENT(RawMonitoringFrameReceived),
                                                              BIND_EVENT(MonitoringFrameReceivedError),
                                                              IScanner::getConfig().hostUDPPortData(),
                                                              IScanner::getConfig().clientIp(),
                                                              DATA_PORT_OF_SCANNER_DEVICE),
                              // Callbacks
                              std::bind(&ScannerV2::scannerStartedCB, this),
                              std::bind(&ScannerV2::scannerStoppedCB, this),
                              IScanner::getLaserScanCB());
}

ScannerV2::ScannerV2(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_cb)
  : IScanner(scanner_config, laser_scan_cb), sm_(new ScannerStateMachine(createStateMachineArgs()))
{
  const std::lock_guard<std::mutex> lock(sm_mutex_);
  sm_->start();
}

ScannerV2::~ScannerV2()
{
  PSENSCAN_DEBUG("Scanner", "Destruction called.");
  stopStartWatchdog();
  const std::lock_guard<std::mutex> lock(sm_mutex_);
  sm_->stop();
}

std::future<void> ScannerV2::start()
{
  PSENSCAN_INFO("Scanner", "Start scanner called.");
  std::future<void> retval_future;
  try
  {
    retval_future = scanner_has_started_.get_future();
  }
  catch (const std::future_error& ex)
  {
    PSENSCAN_ERROR("Scanner", "Start was already called.");
    throw std::runtime_error("Start must not be called twice");
  }
  startStartWatchdog();
  triggerEvent<scanner_events::StartRequest>();
  return retval_future;
}

std::future<void> ScannerV2::stop()
{
  PSENSCAN_INFO("Scanner", "Stop scanner called.");
  std::future<void> retval_future;
  try
  {
    retval_future = scanner_has_stopped_.get_future();
  }
  catch (const std::future_error& ex)
  {
    PSENSCAN_ERROR("Scanner", "Stop was already called.");
    throw std::runtime_error("Stop must not be called twice");
  }
  stopStartWatchdog();
  triggerEvent<scanner_events::StopRequest>();
  return retval_future;
}

void ScannerV2::scannerStartedCB()
{
  PSENSCAN_INFO("ScannerController", "Scanner started successfully.");
  stopStartWatchdog();
  PSENSCAN_DEBUG("Scanner", "Inform user that scanner start is finsihed.");
  scanner_has_started_.set_value();

  // Reinitialize
  scanner_has_started_ = std::promise<void>();
}

void ScannerV2::scannerStoppedCB()
{
  PSENSCAN_INFO("ScannerController", "Scanner stopped successfully.");
  scanner_has_stopped_.set_value();

  // Reinitialize
  scanner_has_stopped_ = std::promise<void>();
}

void ScannerV2::startStartWatchdog()
{
  const std::lock_guard<std::mutex> lock(start_watchdog_mutex_);
  start_watchdog_ = std::make_unique<Watchdog>(REPLY_TIMEOUT, BIND_EVENT(scanner_events::StartTimeout));
}

void ScannerV2::stopStartWatchdog()
{
  const std::lock_guard<std::mutex> lock(start_watchdog_mutex_);
  start_watchdog_.reset(nullptr);
}

}  // namespace psen_scan_v2
