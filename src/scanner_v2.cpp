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

#include <cassert>
#include <stdexcept>

namespace psen_scan_v2
{
using namespace psen_scan_v2::scanner_protocol::scanner_events;

// clang-format off
#define BIND_EVENT(event_name)\
  std::bind(&ScannerV2::triggerEvent<event_name>, this)

#define BIND_RAW_DATA_EVENT(event_name)\
  [this](const RawData& data, const std::size_t& num_bytes){ triggerEventWithParam(event_name(data, num_bytes)); }
// clang-format on

ScannerV2::WatchdogFactory::WatchdogFactory(ScannerV2* scanner) : IWatchdogFactory(), scanner_(scanner)
{
  assert(scanner);
}

std::unique_ptr<Watchdog> ScannerV2::WatchdogFactory::create(const Watchdog::Timeout& timeout,
                                                             const std::string& event_type)
{
  if (event_type == "StartReplyTimeout")
  {
    return std::unique_ptr<Watchdog>(
        new Watchdog(timeout, std::bind(&ScannerV2::triggerEvent<scanner_events::StartTimeout>, scanner_)));
  }
  if (event_type == "MonitoringFrameTimeout")
  {
    return std::unique_ptr<Watchdog>(
        new Watchdog(timeout, std::bind(&ScannerV2::triggerEvent<scanner_events::MonitoringFrameTimeout>, scanner_)));
  }

  // LCOV_EXCL_START
  throw std::runtime_error("WatchdogFactory called with event for which no creation process existiert.");
  // LCOV_EXCL_STOP
}

StateMachineArgs* ScannerV2::createStateMachineArgs()
{
  return new StateMachineArgs(IScanner::getConfig(),
                              // LCOV_EXCL_START
                              // The following includes calls to std::bind which are not marked correctly
                              // by some gcc versions, see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=96006
                              // UDP clients
                              std::make_unique<UdpClientImpl>(BIND_RAW_DATA_EVENT(RawReplyReceived),
                                                              BIND_EVENT(ReplyReceiveError),
                                                              IScanner::getConfig().hostUDPPortControl(),
                                                              IScanner::getConfig().clientIp(),
                                                              IScanner::getConfig().scannerControlPort()),
                              std::make_unique<UdpClientImpl>(BIND_RAW_DATA_EVENT(RawMonitoringFrameReceived),
                                                              BIND_EVENT(MonitoringFrameReceivedError),
                                                              IScanner::getConfig().hostUDPPortData(),
                                                              IScanner::getConfig().clientIp(),
                                                              IScanner::getConfig().scannerDataPort()),
                              // Callbacks
                              std::bind(&ScannerV2::scannerStartedCB, this),
                              std::bind(&ScannerV2::scannerStoppedCB, this),
                              // LCOV_EXCL_STOP
                              IScanner::getLaserScanCB(),
                              std::unique_ptr<IWatchdogFactory>(new WatchdogFactory(this)));
}  // namespace psen_scan_v2

ScannerV2::ScannerV2(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_cb)
  : IScanner(scanner_config, laser_scan_cb), sm_(new ScannerStateMachine(createStateMachineArgs()))
{
  const std::lock_guard<std::mutex> lock(member_mutex_);
  sm_->start();
}

ScannerV2::~ScannerV2()
{
  PSENSCAN_DEBUG("Scanner", "Destruction called.");

  const std::lock_guard<std::mutex> lock(member_mutex_);
  sm_->stop();
}

std::future<void> ScannerV2::start()
{
  PSENSCAN_INFO("Scanner", "Start scanner called.");

  const std::lock_guard<std::mutex> lock(member_mutex_);
  if (scanner_has_started_)
  {
    return std::future<void>();
  }

  // No call to triggerEvent() because lock already taken
  sm_->process_event(scanner_events::StartRequest());
  // Due to the fact that the getting of the future should always succeed (because of the
  // protected check and replacement of the promise), the std::future_error exception is not caught here.
  scanner_has_started_ = std::promise<void>();
  return scanner_has_started_.value().get_future();
}

std::future<void> ScannerV2::stop()
{
  PSENSCAN_INFO("Scanner", "Stop scanner called.");

  const std::lock_guard<std::mutex> lock(member_mutex_);
  if (scanner_has_stopped_)
  {
    return std::future<void>();
  }

  // No call to triggerEvent() because lock already taken
  sm_->process_event(scanner_events::StopRequest());
  // Due to the fact that the getting of the future should always succeed (because of the
  // protected check and replacement of the promise), the std::future_error exception is not caught here.
  scanner_has_stopped_ = std::promise<void>();
  return scanner_has_stopped_.value().get_future();
}

// PLEASE NOTE:
// The callback does not take a member lock because the callback is always called
// via call to triggerEvent() or triggerEventWithParam() which already take the mutex.
void ScannerV2::scannerStartedCB()
{
  PSENSCAN_INFO("ScannerController", "Scanner started successfully.");
  scanner_has_started_.value().set_value();
  scanner_has_started_ = boost::none;
}

// PLEASE NOTE:
// The callback does not take a member lock because the callback is always called
// via call to triggerEvent() or triggerEventWithParam() which already take the mutex.
void ScannerV2::scannerStoppedCB()
{
  PSENSCAN_INFO("ScannerController", "Scanner stopped successfully.");
  scanner_has_stopped_.value().set_value();
  scanner_has_stopped_ = boost::none;
}

}  // namespace psen_scan_v2
