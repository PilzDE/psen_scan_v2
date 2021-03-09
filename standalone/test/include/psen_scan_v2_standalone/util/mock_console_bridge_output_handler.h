// Copyright (c) 2019-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_MOCK_CONSOLE_BRIDGE_OUTPUT_HANDLER_H
#define PSEN_SCAN_V2_MOCK_CONSOLE_BRIDGE_OUTPUT_HANDLER_H

#include <gmock/gmock.h>
#include <console_bridge/console.h>

namespace psen_scan_v2_standalone_test
{
class MockConsoleBridgeOutputHandler : public console_bridge::OutputHandler
{
public:
  MOCK_METHOD4(log, void(const std::string&, console_bridge::LogLevel, const char*, int));
};

#define INJECT_LOG_MOCK                                                                                                \
  MockConsoleBridgeOutputHandler mock;                                                                                 \
  console_bridge::useOutputHandler(&mock);

#define INJECT_NICE_LOG_MOCK                                                                                           \
  ::testing::NiceMock<MockConsoleBridgeOutputHandler> mock;                                                            \
  console_bridge::useOutputHandler(&mock);

#define REMOVE_LOG_MOCK console_bridge::restorePreviousOutputHandler();

#define EXPECT_LOG(level, msg, file, line)                                                                             \
  EXPECT_CALL(mock, log(msg, console_bridge::CONSOLE_BRIDGE_LOG_##level, file, line))

#define EXPECT_LOG_SHORT(level, msg)                                                                                   \
  EXPECT_CALL(mock, log(msg, console_bridge::CONSOLE_BRIDGE_LOG_##level, ::testing::_, ::testing::_))

#define EXPECT_ANY_LOG() EXPECT_CALL(mock, log(::testing::_, ::testing::_, ::testing::_, ::testing::_))

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_MOCK_CONSOLE_BRIDGE_OUTPUT_HANDLER_H
