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
#ifndef PSEN_SCAN_V2_SCANNER_V2_H
#define PSEN_SCAN_V2_SCANNER_V2_H

#include <stdexcept>
#include <memory>
#include <mutex>
#include <future>
#include <chrono>
#include <functional>

#include "psen_scan_v2/scanner_interface.h"
#include "psen_scan_v2/scanner_events.h"
#include "psen_scan_v2/scanner_state_machine.h"
#include "psen_scan_v2/laserscan.h"

#include "psen_scan_v2/udp_client.h"
#include "psen_scan_v2/raw_scanner_data.h"

#include "psen_scan_v2/watchdog.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2::scanner_protocol;
using std::placeholders::_1;
using std::placeholders::_2;

// TODO: Move to ScannerController class and read from ScannerConfiguration
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };

static constexpr std::chrono::milliseconds REPLY_TIMEOUT{ 1000 };

class ScannerV2 : public IScanner
{
public:
  ScannerV2(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_cb);
  ~ScannerV2();

public:
  std::future<void> start() override;
  std::future<void> stop() override;

private:
  // Raw pointer used here because "msm::back::state_machine" cannot properly pass
  // a "std::unique_ptr" to "msm::front::state_machine_def".
  StateMachineArgs* createStateMachineArgs();

  template <class T>
  void triggerEventWithParam(const T& event);

  template <class T>
  void triggerEvent();

  void scannerStartedCB();
  void scannerStoppedCB();

  void startStartWatchdog();
  void stopStartWatchdog();

private:
  std::promise<void> scanner_has_started_;
  std::promise<void> scanner_has_stopped_;

  // The watchdog pointer is changed by the user-main-thread and by the UDP client callback-thread/io-service-thread,
  // and, therefore, needs to be protected/sychronized via mutex.
  std::mutex start_watchdog_mutex_;
  std::unique_ptr<Watchdog> start_watchdog_{};

  std::mutex sm_mutex_;
  std::unique_ptr<ScannerStateMachine> sm_;
};

template <class T>
void ScannerV2::triggerEventWithParam(const T& event)
{
  const std::lock_guard<std::mutex> lock(sm_mutex_);
  sm_->process_event(event);
}

template <class T>
void ScannerV2::triggerEvent()
{
  triggerEventWithParam<T>(T());
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_V2_H
