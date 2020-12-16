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

#include <string>
#include <functional>
#include <chrono>
#include <memory>

#include <boost/asio.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/async_barrier.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/mock_udp_server.h"
#include "psen_scan_v2/udp_client.h"

using namespace psen_scan_v2;
using namespace ::testing;
using boost::asio::ip::udp;

namespace psen_scan_v2_test
{
static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short HOST_UDP_PORT{ 46001 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short UDP_MOCK_PORT{ HOST_UDP_PORT + 1 };

static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 5 };

using std::placeholders::_1;
using std::placeholders::_2;

class UdpClientTests : public testing::Test
{
public:
  UdpClientTests();
  MOCK_METHOD2(handleNewData, void(const RawData&, const std::size_t&));
  MOCK_METHOD1(handleError, void(const std::string&));

  MOCK_METHOD2(receivedUdpMsg, void(const udp::endpoint&, const psen_scan_v2::RawData&));

public:
  void sendTestDataToClient();
  void sendEmptyTestDataToClient();

protected:
  MockUDPServer mock_udp_server_{ UDP_MOCK_PORT, std::bind(&UdpClientTests::receivedUdpMsg, this, _1, _2) };

  std::unique_ptr<UdpClientImpl> udp_client_{ new UdpClientImpl(std::bind(&UdpClientTests::handleNewData, this, _1, _2),
                                                                std::bind(&UdpClientTests::handleError, this, _1),
                                                                HOST_UDP_PORT,
                                                                inet_network(UDP_MOCK_IP_ADDRESS.c_str()),
                                                                UDP_MOCK_PORT) };

  const RawData send_array_{ 'H', 'e', 'l', 'l', 'o' };
  const udp::endpoint host_endpoint;
};

UdpClientTests::UdpClientTests()
  : host_endpoint(udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), HOST_UDP_PORT))
{
}

void UdpClientTests::sendTestDataToClient()
{
  mock_udp_server_.asyncSend(host_endpoint, send_array_);
}

void UdpClientTests::sendEmptyTestDataToClient()
{
  const psen_scan_v2::RawData data;
  assert(data.empty());
  mock_udp_server_.asyncSend(host_endpoint, data);
}

ACTION_P(OpenBarrier, barrier)
{
  barrier->release();
}

TEST_F(UdpClientTests, testAsyncReadOperation)
{
  Barrier client_received_data_barrier;
  EXPECT_CALL(*this, handleNewData(_, send_array_.size())).WillOnce(OpenBarrier(&client_received_data_barrier));

  udp_client_->startAsyncReceiving();
  sendTestDataToClient();
  EXPECT_TRUE(client_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Udp client did not receive data";
}

TEST_F(UdpClientTests, testSingleAsyncReadOperation)
{
  Barrier client_received_data_barrier;
  EXPECT_CALL(*this, handleNewData(_, send_array_.size())).WillOnce(OpenBarrier(&client_received_data_barrier));

  udp_client_->startAsyncReceiving(ReceiveMode::single);
  sendTestDataToClient();
  EXPECT_TRUE(client_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Udp client did not receive data";
}

TEST_F(UdpClientTests, Should_NotCallErrorHandler_WhenDestroyedWhileAsyncReceivePending)
{
  EXPECT_CALL(*this, handleError(_)).Times(0);

  udp_client_->startAsyncReceiving();
  udp_client_->close();
}

TEST_F(UdpClientTests, testErrorHandlingForReceive)
{
  Barrier error_handler_called_barrier;
  EXPECT_CALL(*this, handleError(_)).WillOnce(OpenBarrier(&error_handler_called_barrier));

  udp_client_->startAsyncReceiving(ReceiveMode::single);
  sendEmptyTestDataToClient();
  EXPECT_TRUE(error_handler_called_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Error handler should have been called";
}

TEST_F(UdpClientTests, testWriteOperation)
{
  std::string str = "Hello!";
  RawData write_buf;
  std::copy(str.begin(), str.end(), std::back_inserter(write_buf));

  Barrier server_mock_received_data_barrier;
  EXPECT_CALL(*this, receivedUdpMsg(_, write_buf)).WillOnce(OpenBarrier(&server_mock_received_data_barrier));

  mock_udp_server_.asyncReceive();
  udp_client_->write(write_buf);

  EXPECT_TRUE(server_mock_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Server mock did not receive data";
}

TEST_F(UdpClientTests, testWritingWhileReceiving)
{
  std::string str = "Hello!";
  RawData write_buf;
  std::copy(str.begin(), str.end(), std::back_inserter(write_buf));

  Barrier client_received_data_barrier;
  Barrier server_mock_received_data_barrier;
  EXPECT_CALL(*this, handleNewData(_, send_array_.size())).WillOnce(OpenBarrier(&client_received_data_barrier));
  EXPECT_CALL(*this, receivedUdpMsg(_, write_buf))
      .WillOnce(DoAll(InvokeWithoutArgs(this, &UdpClientTests::sendTestDataToClient),
                      OpenBarrier(&server_mock_received_data_barrier)));

  mock_udp_server_.asyncReceive();

  udp_client_->startAsyncReceiving();

  udp_client_->write(write_buf);

  EXPECT_TRUE(server_mock_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Server mock did not receive data";
  EXPECT_TRUE(client_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Udp client did not receive data";
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
