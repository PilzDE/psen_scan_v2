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

StateMachineArgs* ScannerV2::createStateMachineArgs(const unsigned short& data_port_scanner,
                                                    const unsigned short& control_port_scanner)
{
  return new StateMachineArgs(IScanner::getConfig(),
                              // UDP clients
                              std::make_unique<UdpClientImpl>(BIND_RAW_DATA_EVENT(RawReplyReceived),
                                                              BIND_EVENT(ReplyReceiveError),
                                                              IScanner::getConfig().hostUDPPortControl(),
                                                              IScanner::getConfig().clientIp(),
                                                              control_port_scanner),
                              std::make_unique<UdpClientImpl>(BIND_RAW_DATA_EVENT(RawMonitoringFrameReceived),
                                                              BIND_EVENT(MonitoringFrameReceivedError),
                                                              IScanner::getConfig().hostUDPPortData(),
                                                              IScanner::getConfig().clientIp(),
                                                              data_port_scanner),
                              // Callbacks
                              std::bind(&ScannerV2::scannerStartedCB, this),
                              std::bind(&ScannerV2::scannerStoppedCB, this),
                              IScanner::getLaserScanCB());
}  // namespace psen_scan_v2

ScannerV2::ScannerV2(const ScannerConfiguration& scanner_config,
                     const LaserScanCallback& laser_scan_cb,
                     const unsigned short data_port_scanner,
                     const unsigned short control_port_scanner)
  : IScanner(scanner_config, laser_scan_cb)
  , sm_(new ScannerStateMachine(createStateMachineArgs(data_port_scanner, control_port_scanner)))
{
  const std::lock_guard<std::recursive_mutex> lock(member_mutex_);
  sm_->start();
}

ScannerV2::~ScannerV2()
{
  PSENSCAN_DEBUG("Scanner", "Destruction called.");

  const std::lock_guard<std::recursive_mutex> lock(member_mutex_);
  start_watchdog_.reset();  // Stops watchdog from running
  sm_->stop();
}

std::future<void> ScannerV2::start()
{
  PSENSCAN_INFO("Scanner", "Start scanner called.");

  const std::lock_guard<std::recursive_mutex> lock(member_mutex_);
  std::future<void> retval;
  if (!scanner_has_started_)
  {
    scanner_has_started_ = std::promise<void>();
    // Due to the fact that the getting of the future should always succeed (because of the
    // protected check and replacement of the promise), the std::future_error exception is not caught here.
    retval = scanner_has_started_.value().get_future();
  }
  else
  {
    return std::future<void>();
  }
  // Start watchdog
  start_watchdog_ = std::make_unique<Watchdog>(REPLY_TIMEOUT, BIND_EVENT(scanner_events::StartTimeout));
  triggerEvent<scanner_events::StartRequest>();
  return retval;
}

std::future<void> ScannerV2::stop()
{
  PSENSCAN_INFO("Scanner", "Stop scanner called.");

  const std::lock_guard<std::recursive_mutex> lock(member_mutex_);
  std::future<void> retval;
  if (!scanner_has_stopped_)
  {
    scanner_has_stopped_ = std::promise<void>();
    // Due to the fact that the getting of the future should always succeed (because of the
    // protected check and replacement of the promise), the std::future_error exception is not caught here.
    retval = scanner_has_stopped_.value().get_future();
  }
  else
  {
    return std::future<void>();
  }
  start_watchdog_.reset();  // Stops watchdog from running
  triggerEvent<scanner_events::StopRequest>();
  return retval;
}

void ScannerV2::scannerStartedCB()
{
  PSENSCAN_INFO("ScannerController", "Scanner started successfully.");

  const std::lock_guard<std::recursive_mutex> lock(member_mutex_);
  start_watchdog_.reset();  // Stops watchdog from running
  scanner_has_started_.value().set_value();
  scanner_has_started_ = boost::none;
}

void ScannerV2::scannerStoppedCB()
{
  PSENSCAN_INFO("ScannerController", "Scanner stopped successfully.");

  const std::lock_guard<std::recursive_mutex> lock(member_mutex_);
  scanner_has_stopped_.value().set_value();
  scanner_has_stopped_ = boost::none;
}

}  // namespace psen_scan_v2
