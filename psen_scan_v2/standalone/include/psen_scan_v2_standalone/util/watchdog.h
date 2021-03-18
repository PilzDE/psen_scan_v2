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
#ifndef PSEN_SCAN_V2_STANDALONE_WATCHDOG_H
#define PSEN_SCAN_V2_STANDALONE_WATCHDOG_H

#include <chrono>
#include <functional>
#include <thread>
#include <atomic>
#include <condition_variable>

#include "psen_scan_v2_standalone/util/async_barrier.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Namespace containing utilities like the watchdog and barrier
 */
namespace util
{
/**
 * @brief Watchdog which continuously calls the specified timeout handler
 *
 * After the specified timeout time has passed the timeout_handler is called and the timer restarts.
 * This continues as long as the watchdog exists.
 */
class Watchdog
{
public:
  using Timeout = const std::chrono::high_resolution_clock::duration;

public:
  Watchdog(const Timeout& timeout, const std::function<void()>& timeout_handler);
  ~Watchdog();

public:
  //! @brief Restarts the watchdog timer.
  void reset();

private:
  /**
   * @returns std::cv_status::timeout if the specified timeout has expired or std::cv_status::no_timeout
   * if the condition variable was notified.
   *
   * @note The function may also return spuriously with std::cv_status::no_timeout even if the condition variable
   * was not notified. For more info see:
   * https://en.cppreference.com/w/cpp/thread/condition_variable/wait_for
   */
  std::cv_status wait_for(const Timeout& timeout);

private:
  util::Barrier thread_startetd_barrier_;
  std::atomic_bool terminated_{ false };
  std::condition_variable cv_;
  std::mutex cv_m_;
  std::thread timer_thread_;
};

inline Watchdog::Watchdog(const Timeout& timeout, const std::function<void()>& timeout_handler)
  : timer_thread_([this, timeout, timeout_handler]() {
    thread_startetd_barrier_.release();
    while (!terminated_)
    {
      if ((this->wait_for(timeout) == std::cv_status::timeout) && !terminated_)
      {
        timeout_handler();
      }
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

inline std::cv_status Watchdog::wait_for(const Timeout& timeout)
{
  std::unique_lock<std::mutex> lk(cv_m_);
  return cv_.wait_for(lk, timeout);
}

inline void Watchdog::reset()
{
  cv_.notify_all();
}

inline Watchdog::~Watchdog()
{
  terminated_ = true;
  // Notify timeout thread to wake up and end execution
  cv_.notify_all();
  if (timer_thread_.joinable())
  {
    timer_thread_.join();
  }
}

}  // namespace util
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_WATCHDOG_H
