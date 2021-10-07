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
#define EXPECT_FUTURE_IS_READY(future) EXPECT_EQ(future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready)

#define EXPECT_FUTURE_TIMEOUT(future, wait_timeout)                                                                    \
  EXPECT_EQ(future.wait_for(wait_timeout), std::future_status::timeout)

#define EXPECT_BARRIER_OPENS(barrier, wait_timeout) EXPECT_TRUE(barrier.waitTillRelease(wait_timeout))

#define EXPECT_DOES_NOT_BLOCK(statement)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto future = std::async(std::launch::async, [&]() { statement });                                           \
    EXPECT_FUTURE_IS_READY(future) << #statement << " does not return.";                                               \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define EXPECT_STOP_REQUEST_CALL(hw_mock)                                                                              \
  EXPECT_CALL(hw_mock, receiveControlMsg(_, data_conversion_layer::stop_request::serialize()))

#define EXPECT_START_REQUEST_CALL(hw_mock, config)                                                                     \
  EXPECT_CALL(                                                                                                         \
      hw_mock,                                                                                                         \
      receiveControlMsg(                                                                                               \
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config))))

#define EXPECT_CALLBACK_WILL_OPEN_BARRIER(cb, msgs, barrier)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto timestamp{ util::getCurrentTime() };                                                                    \
    const auto scan{ createReferenceScan(msgs, timestamp) };                                                           \
    EXPECT_CALL(cb, LaserScanCallback(AllOf(ScanDataEqual(scan), TimestampInExpectedTimeframe(scan, timestamp))))      \
        .WillOnce(OpenBarrier(&barrier));                                                                              \
  } while (false)

#define EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock, driver, config)                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier start_req_barrier;                                                                                   \
    std::future<void> start_future;                                                                                    \
    EXPECT_START_REQUEST_CALL(*hw_mock, *config).WillOnce(OpenBarrier(&start_req_barrier));                            \
    EXPECT_DOES_NOT_BLOCK(start_future = driver->start(););                                                            \
    EXPECT_BARRIER_OPENS(start_req_barrier, DEFAULT_TIMEOUT) << "Start request not received";                          \
    hw_mock->sendStartReply();                                                                                         \
    EXPECT_FUTURE_IS_READY(start_future) << "Scanner::start() not finished";                                           \
  } while (false)

#define EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock, driver)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier stop_req_barrier;                                                                                    \
    std::future<void> stop_future;                                                                                     \
    EXPECT_STOP_REQUEST_CALL(*hw_mock).WillOnce(OpenBarrier(&stop_req_barrier));                                       \
    EXPECT_DOES_NOT_BLOCK(stop_future = driver->stop(););                                                              \
    EXPECT_BARRIER_OPENS(stop_req_barrier, DEFAULT_TIMEOUT) << "Stop request not received";                            \
    hw_mock->sendStopReply();                                                                                          \
    EXPECT_FUTURE_IS_READY(stop_future) << "Scanner::stop() not finished";                                             \
  } while (false)

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_EXPECTATIONS_H
