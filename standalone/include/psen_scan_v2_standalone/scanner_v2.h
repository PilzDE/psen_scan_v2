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
#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_V2_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_V2_H

#include <memory>
#include <mutex>
#include <future>
#include <functional>

#include <boost/optional.hpp>

#include "psen_scan_v2_standalone/scanner_interface.h"
#include "psen_scan_v2_standalone/protocol_layer/scanner_events.h"
#include "psen_scan_v2_standalone/protocol_layer/scanner_state_machine.h"

#include "psen_scan_v2_standalone/util/watchdog.h"

/**
 * @brief Root namespace in which the software components to communicate with the scanner (firmware-version: 2)
 * are realised/implemented.
 */
namespace psen_scan_v2_standalone
{
class ScannerConfiguration;
using namespace psen_scan_v2_standalone::protocol_layer;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief This is the implementation of the Scanner API defined by IScanner.
 *
 * The class ScannerV2 is responsible for the initialization of:
 * - the state machine.
 * - the Udp connections.
 * - the guards to ensure threads save interaction between the user, udp connections and timeouts.
 *
 * It uses the passed ScannerConfiguration for all configurable parts of this process.
 *
 * The class creates two UdpClientImpl, a WatchdogFactory and passes them together with the @ref LaserScanCallback to
 * the scanner_protocol::ScannerStateMachine via scanner_protocol::StateMachineArgs.
 *
 * @see IScanner
 * @see communication_layer::UdpClientImpl
 * @see protocol_layer::ScannerStateMachine
 * @see ScannerConfiguration
 */
class ScannerV2 : public IScanner
{
public:
  ScannerV2(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback);
  ~ScannerV2();

public:
  //! @brief An exception is set in the returned future if the scanner start was not successful.
  std::future<void> start() override;
  std::future<void> stop() override;

private:
  template <class T>
  void triggerEventWithParam(const T& event);

  template <class T>
  void triggerEvent();

  void scannerStartedCallback();
  void scannerStoppedCallback();
  void scannerStartErrorCallback(const std::string& error_msg);

private:
  using OptionalPromise = boost::optional<std::promise<void>>;

private:
  OptionalPromise scanner_has_started_{ boost::none };
  OptionalPromise scanner_has_stopped_{ boost::none };

  //! @brief This Mutex protects ALL members of the Scanner against concurrent access.
  //! So far there exist at least the following threads, potentially causing concurrent access to the members:
  //! - user-main-thread
  //! - io_service thread of UDPClient
  //! - watchdog threads
  std::mutex member_mutex_;

  std::unique_ptr<ScannerStateMachine> sm_;
};

template <class T>
void ScannerV2::triggerEventWithParam(const T& event)
{
  const std::lock_guard<std::mutex> lock(member_mutex_);
  sm_->process_event(event);
}

template <class T>
void ScannerV2::triggerEvent()
{
  triggerEventWithParam<T>(T());
}

}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_V2_H
