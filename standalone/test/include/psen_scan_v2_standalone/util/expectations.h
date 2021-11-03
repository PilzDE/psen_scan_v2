// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_EXPECTATIONS_H
#define PSEN_SCAN_V2_STANDALONE_TEST_EXPECTATIONS_H

#include <chrono>
#include <future>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

namespace psen_scan_v2_standalone_test
{
#define EXPECT_FUTURE_IS_READY(future, wait_timeout) EXPECT_EQ(future.wait_for(wait_timeout), std::future_status::ready)

#define EXPECT_FUTURE_TIMEOUT(future, wait_timeout)                                                                    \
  EXPECT_EQ(future.wait_for(wait_timeout), std::future_status::timeout)

#define EXPECT_BARRIER_OPENS(barrier, wait_timeout) EXPECT_TRUE(barrier.waitTillRelease(wait_timeout))

#define EXPECT_DOES_NOT_BLOCK(statement)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto future = std::async(std::launch::async, [&]() { statement });                                           \
    EXPECT_FUTURE_IS_READY(future, std::chrono::seconds{ 1 }) << #statement << " does not return.";                    \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define EXPECT_CALL_AND_WAIT(mock, call, timeout)                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    psen_scan_v2_standalone::util::Barrier msg_received_barrier;                                                       \
    EXPECT_CALL(mock, call).WillOnce(OpenBarrier(&msg_received_barrier));                                              \
    msg_received_barrier.waitTillRelease(timeout);                                                                     \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define EXPECT_CALLS_AND_WAIT(mock, call, timeout)                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    psen_scan_v2_standalone::util::Barrier msg_received_barrier;                                                       \
    EXPECT_CALL(mock, call).WillOnce(OpenBarrier(&msg_received_barrier)).WillRepeatedly(Return());                     \
    msg_received_barrier.waitTillRelease(timeout);                                                                     \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define EXPECT_CALL_X_TIMES_RUN_STATEMENT_AND_WAIT(mock, call, times, statement, timeout)                              \
  do                                                                                                                   \
  {                                                                                                                    \
    psen_scan_v2_standalone::util::Barrier msg_received_barrier;                                                       \
    EXPECT_CALL(mock, call).WillOnce(OpenBarrier(&msg_received_barrier));                                              \
    EXPECT_CALL(mock, call).Times(times - 1).RetiresOnSaturation();                                                    \
    statement;                                                                                                         \
    msg_received_barrier.waitTillRelease(timeout);                                                                     \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define EXPECT_CALL_WITH_BARRIER(barrier_name, mock, call)                                                             \
  psen_scan_v2_standalone::util::Barrier barrier_name;                                                                 \
  EXPECT_CALL(mock, call).WillOnce(OpenBarrier(&barrier_name));

#define EXPECT_CALLS_ON_ASYNC_STATEMENT_AND_WAIT(mock, statement, timeout, ...)                                        \
  do                                                                                                                   \
  {                                                                                                                    \
    std::vector<psen_scan_v2_standalone::util::Barrier> barriers;                                                      \
    FOR_EACH()(auto call : calls)                                                                                      \
    {                                                                                                                  \
      barriers.push_back(psen_scan_v2_standalone::util::Barrier);                                                      \
      EXPECT_CALL(mock, call).WillOnce(OpenBarrier(&barriers.back()));                                                 \
    }                                                                                                                  \
    statement;                                                                                                         \
    for (auto barrier : barriers)                                                                                      \
    {                                                                                                                  \
      barrier.waitTillRelease(timeout);                                                                                \
    }                                                                                                                  \
  } while (false)

}  // namespace psen_scan_v2_standalone_test

#define EXPECT_THROW_AND_WHAT(statement, expected_exception, expected_message)                                         \
  EXPECT_THROW(                                                                                                        \
      {                                                                                                                \
        try                                                                                                            \
        {                                                                                                              \
          statement;                                                                                                   \
        }                                                                                                              \
        catch (const expected_exception& e)                                                                            \
        {                                                                                                              \
          EXPECT_STREQ(expected_message, e.what());                                                                    \
          throw;                                                                                                       \
        }                                                                                                              \
      },                                                                                                               \
      expected_exception);

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_EXPECTATIONS_H
