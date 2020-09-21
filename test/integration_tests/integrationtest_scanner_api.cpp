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
#include "psen_scan_v2/mock_udp_server.h"

// Software under testing
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/scanner_reply_msg.h"

namespace psen_scan_v2_test
{
static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short SCANNER_UDP_PORT_CONTROL{ 3000 };

static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr uint32_t HOST_UDP_PORT_DATA{ 45000 };
static constexpr uint32_t HOST_UDP_PORT_CONTROL{ 57000 };

static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ 1. };

static constexpr std::chrono::milliseconds WAIT_TIMEOUT{ 10 };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0u };

using namespace psen_scan_v2;
using namespace ::testing;

class UserCallbacks
{
public:
  MOCK_METHOD1(LaserScanCallback, void(const LaserScan&));
};

class ScannerMock : public MockUDPServer
{
public:
  ScannerMock();

public:
  void sendStartReply();
  void sendStopReply();

private:
  void sendReply(const uint32_t reply_type);

private:
  const udp::endpoint host_endpoint_{ udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS),
                                                    HOST_UDP_PORT_CONTROL) };
};

ScannerMock::ScannerMock() : MockUDPServer(SCANNER_UDP_PORT_CONTROL)
{
}

void ScannerMock::sendReply(const uint32_t reply_type)
{
  const ScannerReplyMsg msg(reply_type, 0x00);
  asyncSend<REPLY_MSG_FROM_SCANNER_SIZE>(host_endpoint_, msg.toRawData());
}

void ScannerMock::sendStartReply()
{
  sendReply(getOpCodeValue(ScannerReplyMsgType::Start));
}

void ScannerMock::sendStopReply()
{
  sendReply(getOpCodeValue(ScannerReplyMsgType::Stop));
}

ScannerConfiguration createScannerConfig()
{
  return ScannerConfiguration(
      HOST_IP_ADDRESS, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, SCANNER_IP_ADDRESS, START_ANGLE, END_ANGLE);
}

TEST(ScannerAPITests, testStartFunctionality)
{
  ScannerMock scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));
  const StartRequest start_req(config, DEFAULT_SEQ_NUMBER);

  Barrier start_req_received_barrier;
  EXPECT_CALL(scanner_mock, receivedUdpMsg(_, start_req.toRawData()))
      .WillOnce(InvokeWithoutArgs([&start_req_received_barrier]() { start_req_received_barrier.release(); }));

  scanner_mock.asyncReceive();
  const auto start_future{ std::async(std::launch::async, [&scanner]() { scanner.start(); }) };

  EXPECT_TRUE(start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";
  EXPECT_EQ(start_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::start() finished too early";
  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST(ScannerAPITests, testStopFunctionality)
{
  ScannerMock scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  EXPECT_CALL(scanner_mock, receivedUdpMsg(_, StartRequest(config, DEFAULT_SEQ_NUMBER).toRawData()))
      .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  Barrier stop_req_received_barrier;
  EXPECT_CALL(scanner_mock, receivedUdpMsg(_, StopRequest().toRawData()))
      .WillOnce(InvokeWithoutArgs([&stop_req_received_barrier]() { stop_req_received_barrier.release(); }));

  scanner_mock.asyncReceive();
  scanner.start();

  scanner_mock.asyncReceive();
  const auto stop_future{ std::async(std::launch::async, [&scanner]() { scanner.stop(); }) };

  EXPECT_TRUE(stop_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Stop request not received";
  EXPECT_EQ(stop_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::stop() finished too early";
  scanner_mock.sendStopReply();
  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::stop() not finished";
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
