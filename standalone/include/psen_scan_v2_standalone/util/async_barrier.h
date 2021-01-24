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
#ifndef PSEN_SCAN_V2_STANDALONE_ASYNC_BARRIER_H
#define PSEN_SCAN_V2_STANDALONE_ASYNC_BARRIER_H

#include <future>
#include <thread>
#include <chrono>

namespace psen_scan_v2_standalone
{
namespace util
{
/**
 * @brief Helper class to simplify the synchronization between different threads.
 */
class Barrier
{
public:
  void release();
  template <class Rep, class Period>
  bool waitTillRelease(const std::chrono::duration<Rep, Period>& timeout) const;

private:
  std::promise<void> barrier_;
  const std::future<void> future_{ barrier_.get_future() };
};

inline void util::Barrier::release()
{
  barrier_.set_value();
}

template <class Rep, class Period>
inline bool util::Barrier::waitTillRelease(const std::chrono::duration<Rep, Period>& timeout) const
{
  return future_.wait_for(timeout) == std::future_status::ready;
}

}  // namespace util
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_ASYNC_BARRIER_H
