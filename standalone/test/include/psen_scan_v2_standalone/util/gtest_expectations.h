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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_GTEST_EXPECTATIONS_H
#define PSEN_SCAN_V2_STANDALONE_TEST_GTEST_EXPECTATIONS_H

#include <algorithm>
#include <chrono>
#include <future>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/util/format_range.h"

#define EXPECT_FUTURE_IS_READY(future, wait_timeout) EXPECT_EQ(future.wait_for(wait_timeout), std::future_status::ready)

#define EXPECT_FUTURE_TIMEOUT(future, wait_timeout)                                                                    \
  EXPECT_EQ(future.wait_for(wait_timeout), std::future_status::timeout)

#define EXPECT_BARRIER_OPENS(barrier, wait_timeout) EXPECT_TRUE(barrier.waitTillRelease(wait_timeout))

#define EXPECT_NO_BLOCK_NO_THROW(statement)                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    using namespace std::chrono_literals;                                                                              \
    auto future = std::async(std::launch::async, [&]() { statement });                                                 \
    EXPECT_TRUE(future.valid());                                                                                       \
    EXPECT_FUTURE_IS_READY(future, std::chrono::seconds(2)) << #statement << " does not return.";                      \
    EXPECT_NO_THROW(future.get();) << #statement << " does throw an exception.";                                       \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

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

#define EXPECT_IO_STATE_EQ_IO_PIN(io_state, io_pin_data, start_index)                                                  \
  EXPECT_EQ(io_pin_data.logical_input, io_state.at(start_index).logicalInput());                                       \
  EXPECT_EQ(io_pin_data.output, io_state.at(start_index).output());                                                    \

#define EXPECT_CONTAINER_UNORDERED_EQ(var1, var2)                                                                      \
  EXPECT_EQ(var1.size(), var2.size());                                                                                 \
  for (const auto& v : var2)                                                                                           \
  {                                                                                                                    \
    EXPECT_NE(std::find(var1.begin(), var1.end(), v), var1.end())                                                      \
        << "Did not find the expected element: " << v << " in " << util::formatRange(var1);                            \
  }

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_GTEST_EXPECTATIONS_H
