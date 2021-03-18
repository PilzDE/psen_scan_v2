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

#include <chrono>
#include <console_bridge/console.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/util/logging.h"

#include "psen_scan_v2_standalone/util/mock_console_bridge_output_handler.h"

using namespace psen_scan_v2_standalone_test;

TEST(LoggingTest, logInfo)
{
  INJECT_LOG_MOCK;
  setLogLevel(CONSOLE_BRIDGE_LOG_INFO);
  EXPECT_LOG(INFO, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_INFO("Name", "msg");
}

TEST(LoggingTest, logDebug)
{
  INJECT_LOG_MOCK;
  setLogLevel(CONSOLE_BRIDGE_LOG_DEBUG);
  EXPECT_LOG(DEBUG, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_DEBUG("Name", "msg");
}

TEST(LoggingTest, logWarn)
{
  INJECT_LOG_MOCK;
  EXPECT_LOG(WARN, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_WARN("Name", "msg");
}

TEST(LoggingTest, logError)
{
  INJECT_LOG_MOCK;
  EXPECT_LOG(ERROR, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_ERROR("Name", "msg");
}

TEST(LoggingTest, logInfoThrottle)
{
  INJECT_LOG_MOCK;
  setLogLevel(CONSOLE_BRIDGE_LOG_INFO);
  EXPECT_LOG(INFO, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_INFO_THROTTLE(0.1, "Name", "msg");
}

TEST(LoggingTest, logDebugThrottle)
{
  INJECT_LOG_MOCK;
  setLogLevel(CONSOLE_BRIDGE_LOG_DEBUG);
  EXPECT_LOG(DEBUG, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_DEBUG_THROTTLE(0.1, "Name", "msg");
}

TEST(LoggingTest, logWarnThrottle)
{
  INJECT_LOG_MOCK;
  EXPECT_LOG(WARN, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_WARN_THROTTLE(0.1, "Name", "msg");
}

TEST(LoggingTest, logErrorThrottle)
{
  INJECT_LOG_MOCK;
  EXPECT_LOG(ERROR, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_ERROR_THROTTLE(0.1, "Name", "msg");
}

using system_clock = std::chrono::system_clock;
using time_point = system_clock::time_point;
using duration = system_clock::duration;

TEST(LoggingTest, logThrottleInternal)
{
  INJECT_LOG_MOCK;

  const double period{ 0.1 };
  time_point now{ system_clock::now() };

  EXPECT_LOG(ERROR, "Name: msg", __FILE__, __LINE__ + 3).Times(1);
  for (unsigned int i = 0; i < 2; ++i)
  {
    PSENSCAN_ERROR_THROTTLE_INTERNAL(now, period, "Name", "msg");
    now += std::chrono::milliseconds(99);
  }

  EXPECT_LOG(ERROR, "Name: msg", __FILE__, __LINE__ + 3).Times(2);
  for (unsigned int i = 0; i < 2; ++i)
  {
    PSENSCAN_ERROR_THROTTLE_INTERNAL(now, period, "Name", "msg");
    now += std::chrono::milliseconds(101);
  }
}

TEST(LoggingTest, logThrottleInternalConcurrent)
{
  INJECT_LOG_MOCK;

  const double period1{ 0.1 };
  const double period2{ 0.5 };
  time_point now{ system_clock::now() };

  EXPECT_LOG(ERROR, "Name: msg1", __FILE__, __LINE__ + 4).Times(2);
  EXPECT_LOG(ERROR, "Name: msg2", __FILE__, __LINE__ + 4).Times(1);
  for (unsigned int i = 0; i < 2; ++i)
  {
    PSENSCAN_ERROR_THROTTLE_INTERNAL(now, period1, "Name", "msg1");
    PSENSCAN_ERROR_THROTTLE_INTERNAL(now, period2, "Name", "msg2");
    now += std::chrono::milliseconds(101);
  }
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
