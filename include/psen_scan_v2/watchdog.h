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
#ifndef PSEN_SCAN_V2_WATCHDOG_H
#define PSEN_SCAN_V2_WATCHDOG_H

#include <chrono>
#include <functional>
#include <thread>

#include "psen_scan_v2/async_barrier.h"

namespace psen_scan_v2
{
/**
 * @brief Watchdog which continuously calls the specified timeout handler (after the specified timeout time has passed)
 * as long as the watchdog exists.
 */
class Watchdog
{
public:
  Watchdog(const std::chrono::high_resolution_clock::duration& timeout, const std::function<void()>& timeout_handler);
  ~Watchdog();

private:
  Barrier thread_startetd_barrier_;
  Barrier barrier_;
  std::thread timer_thread_;
};

Watchdog::Watchdog(const std::chrono::high_resolution_clock::duration& timeout,
                   const std::function<void()>& timeout_handler)
  : timer_thread_([this, timeout, timeout_handler]() {
    thread_startetd_barrier_.release();
    // Wait until timeout is reached and then call the timeout handler and
    // restart the wait process.
    // If the barrier is released by the destructor, the timer_thread is going to finish.
    while (!barrier_.waitTillRelease(timeout))
    {
      timeout_handler();
    }
  })
{
  // The timer_thread does not always immediately start because the system schedules threads
  // "at a whim". To ensure that the thread is running after the completion of the constructor,
  // we wait until the first command of the thread is executed.
  if (!thread_startetd_barrier_.waitTillRelease(timeout))
  {
    // Difficult to test because this is a timing problem.
    // LCOV_EXCL_START
    throw std::runtime_error("Timeout while waiting for timer thread to start");
    // LCOV_EXCL_STOP
  }
}

Watchdog::~Watchdog()
{
  barrier_.release();
  if (timer_thread_.joinable())
  {
    timer_thread_.join();
  }
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_WATCHDOG_H
