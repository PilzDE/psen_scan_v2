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
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_EXPECTATIONS_H
