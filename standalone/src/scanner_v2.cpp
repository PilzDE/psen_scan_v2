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

#include "psen_scan_v2_standalone/scanner_v2.h"

#include <cassert>
#include <stdexcept>

#include "psen_scan_v2_standalone/scanner_configuration.h"

namespace psen_scan_v2_standalone
{
using namespace psen_scan_v2_standalone::protocol_layer::scanner_events;

// clang-format off
#define BIND_EVENT(event_name)\
  std::bind(&ScannerV2::triggerEvent<event_name>, this)

#define BIND_RAW_DATA_EVENT(event_name)\
  [this](const data_conversion_layer::RawDataConstPtr& data, const std::size_t& num_bytes, const int64_t& timestamp){ triggerEventWithParam(event_name(data, num_bytes, timestamp)); }
// clang-format on

std::unique_ptr<util::Watchdog> WatchdogFactory::create(const util::Watchdog::Timeout& timeout,
                                                        const std::function<void()>& timeout_callback)
{
  return std::unique_ptr<util::Watchdog>(new util::Watchdog(timeout, timeout_callback));
}

ScannerV2::ScannerV2(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback)
  : IScanner(scanner_config, laser_scan_callback)
  , sm_(new ScannerStateMachine(IScanner::config(),
                                // LCOV_EXCL_START
                                // The following includes calls to std::bind which are not marked correctly
                                // by some gcc versions, see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=96006
                                BIND_RAW_DATA_EVENT(RawReplyReceived),
                                BIND_EVENT(ReplyReceiveError),
                                std::bind(&ScannerV2::scannerStartErrorCallback, this, std::placeholders::_1),
                                std::bind(&ScannerV2::scannerStopErrorCallback, this, std::placeholders::_1),
                                BIND_RAW_DATA_EVENT(RawMonitoringFrameReceived),
                                BIND_EVENT(MonitoringFrameReceivedError),
                                std::bind(&ScannerV2::scannerStartedCallback, this),
                                std::bind(&ScannerV2::scannerStoppedCallback, this),
                                IScanner::laserScanCallback(),
                                BIND_EVENT(scanner_events::StartTimeout),
                                BIND_EVENT(scanner_events::MonitoringFrameTimeout)))
// LCOV_EXCL_STOP
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
void ScannerV2::scannerStartedCallback()
{
  PSENSCAN_INFO("ScannerController", "Scanner started successfully.");
  scanner_has_started_.value().set_value();
  scanner_has_started_ = boost::none;
}

// PLEASE NOTE:
// The callback does not take a member lock because the callback is always called
// via call to triggerEvent() or triggerEventWithParam() which already take the mutex.
void ScannerV2::scannerStoppedCallback()
{
  PSENSCAN_INFO("ScannerController", "Scanner stopped successfully.");
  scanner_has_stopped_.value().set_value();
  scanner_has_stopped_ = boost::none;
}

void ScannerV2::scannerStartErrorCallback(const std::string& error_msg)
{
  PSENSCAN_INFO("ScannerController", "Scanner start failed.");
  scanner_has_started_.value().set_exception(std::make_exception_ptr(std::runtime_error(error_msg)));
  scanner_has_started_ = boost::none;
}

void ScannerV2::scannerStopErrorCallback(const std::string& error_msg)
{
  PSENSCAN_INFO("ScannerController", "Scanner stop failed.");
  scanner_has_stopped_.value().set_exception(std::make_exception_ptr(std::runtime_error(error_msg)));
  scanner_has_stopped_ = boost::none;
}

}  // namespace psen_scan_v2_standalone
