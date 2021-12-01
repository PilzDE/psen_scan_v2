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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_GMOCK_EXPECTATIONS_H
#define PSEN_SCAN_V2_STANDALONE_TEST_GMOCK_EXPECTATIONS_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

#define EXPECT_N_ASYNC_CALLS(mock, call, times)                                                                        \
  [&]() {                                                                                                              \
    std::unique_ptr<psen_scan_v2_standalone::util::Barrier> b1{ new psen_scan_v2_standalone::util::Barrier{} };        \
    {                                                                                                                  \
      ::testing::InSequence s;                                                                                         \
      EXPECT_CALL(mock, call).Times(times - 1);                                                                        \
      EXPECT_CALL(mock, call).WillOnce(OpenBarrier(b1.get()));                                                         \
    }                                                                                                                  \
    return b1;                                                                                                         \
  }();

#define EXPECT_ASYNC_CALL(mock, call) EXPECT_N_ASYNC_CALLS(mock, call, 1)

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_GMOCK_EXPECTATIONS_H
