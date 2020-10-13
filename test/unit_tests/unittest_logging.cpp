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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/logging.h"
#include "psen_scan_v2/mock_console_bridge_output_handler.h"

using namespace psen_scan_v2_test;

TEST(LoggingTest, logInfo)
{
  INJECT_LOG_MOCK;
  EXPECT_LOG(INFO, "Name: msg", __FILE__, __LINE__ + 1).Times(1);
  PSENSCAN_INFO("Name", "msg");
}

TEST(LoggingTest, logDebug)
{
  INJECT_LOG_MOCK;
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

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
