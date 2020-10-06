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

#include <future>
#include <thread>
#include <chrono>
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Test frameworks
#include "psen_scan_v2/async_barrier.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"

// Software under testing
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/scan_range.h"
#include "psen_scan_v2/scanner_mock.h"

namespace psen_scan_v2_test
{
using namespace psen_scan_v2;

static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };

static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(1) };

static constexpr std::chrono::milliseconds WAIT_TIMEOUT{ 10 };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0u };

static constexpr double EPS{ 0.001 };

using namespace ::testing;

class UserCallbacks
{
public:
  MOCK_METHOD1(LaserScanCallback, void(const LaserScan&));
};


ScannerConfiguration createScannerConfig()
{
  return ScannerConfiguration(
      HOST_IP_ADDRESS, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, SCANNER_IP_ADDRESS, SCAN_RANGE);
}

TEST(ScannerAPITests, testStartFunctionality)
{
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));
  const StartRequest start_req(config, DEFAULT_SEQ_NUMBER);

  Barrier start_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, start_req.toRawData()))
      .WillOnce(InvokeWithoutArgs([&start_req_received_barrier]() { start_req_received_barrier.release(); }));

  scanner_mock.startListeningForControlMsg();
  const auto start_future{ std::async(std::launch::async, [&scanner]() { scanner.start(); }) };

  EXPECT_TRUE(start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";
  EXPECT_EQ(start_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::start() finished too early";
  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST(ScannerAPITests, testStopFunctionality)
{
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config, DEFAULT_SEQ_NUMBER).toRawData()))
      .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  Barrier stop_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StopRequest().toRawData()))
      .WillOnce(InvokeWithoutArgs([&stop_req_received_barrier]() { stop_req_received_barrier.release(); }));

  scanner_mock.startListeningForControlMsg();
  scanner.start();

  scanner_mock.startListeningForControlMsg();
  const auto stop_future{ std::async(std::launch::async, [&scanner]() { scanner.stop(); }) };

  EXPECT_TRUE(stop_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Stop request not received";
  EXPECT_EQ(stop_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::stop() finished too early";
  scanner_mock.sendStopReply();
  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::stop() not finished";
}

MATCHER_P2(isEqualToRawMeasurements, expected_measures, eps_val, "doesn't match with the expected scan measurements")
{
  const MeasurementData& scan_data{ arg.getMeasurements() };
  const double eps{ eps_val };
  return !std::any_of(expected_measures.cbegin(),
                      expected_measures.cend(),
                      [&scan_data, eps](const std::pair<std::size_t, uint16_t>& index_value_pair) {
                        return std::abs(scan_data.at(index_value_pair.first) - (index_value_pair.second / 1000.)) > eps;
                      });
};

TEST(ScannerAPITests, testReceivingOfMonitoringFrame)
{
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  UDPFrameTestDataWithoutIntensities raw_scan;
  Barrier monitoring_frame_barrier;
  {
    InSequence seq;

    EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config, DEFAULT_SEQ_NUMBER).toRawData()))
        .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

    EXPECT_CALL(cb, LaserScanCallback(isEqualToRawMeasurements(raw_scan.measures, EPS)))
        .WillOnce(InvokeWithoutArgs([&monitoring_frame_barrier]() { monitoring_frame_barrier.release(); }));
  }

  scanner_mock.startListeningForControlMsg();
  scanner.start();

  scanner_mock.sendMonitoringFrame();
  EXPECT_TRUE(monitoring_frame_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
