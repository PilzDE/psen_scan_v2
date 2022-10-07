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

#include <string>
#include <functional>
#include <chrono>
#include <memory>

#include <boost/asio.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/communication_layer/udp_client.h"

#include "psen_scan_v2_standalone/communication_layer/mock_udp_server.h"

using namespace psen_scan_v2_standalone;
using namespace ::testing;
using boost::asio::ip::udp;
using psen_scan_v2_standalone_test::OpenBarrier;

namespace psen_scan_v2_standalone_test
{
static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short HOST_UDP_PORT{ 46001 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short UDP_MOCK_PORT{ HOST_UDP_PORT + 1 };

static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 5 };

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class UdpClientTests : public testing::Test
{
public:
  UdpClientTests();
  MOCK_METHOD3(handleNewData, void(const data_conversion_layer::RawDataConstPtr&, const std::size_t&, const int64_t&));
  MOCK_METHOD1(handleError, void(const std::string&));

  MOCK_METHOD2(receivedUdpMsg,
               void(const udp::endpoint&, const psen_scan_v2_standalone::data_conversion_layer::RawData&));

public:
  void sendTestDataToClient();
  void sendEmptyTestDataToClient();

protected:
  MockUDPServer mock_udp_server_{ UDP_MOCK_PORT, std::bind(&UdpClientTests::receivedUdpMsg, this, _1, _2) };

#if BOOST_VERSION > 107000
  std::unique_ptr<communication_layer::UdpClientImpl> udp_client_{ new communication_layer::UdpClientImpl(
      std::bind(&UdpClientTests::handleNewData, this, _1, _2, _3),
      std::bind(&UdpClientTests::handleError, this, _1),
      HOST_UDP_PORT,
      boost::asio::ip::make_address_v4(UDP_MOCK_IP_ADDRESS.c_str()).to_uint(),
      UDP_MOCK_PORT) };
#elif BOOST_VERSION >= 106900
  std::unique_ptr<communication_layer::UdpClientImpl> udp_client_{ new communication_layer::UdpClientImpl(
      std::bind(&UdpClientTests::handleNewData, this, _1, _2, _3),
      std::bind(&UdpClientTests::handleError, this, _1),
      HOST_UDP_PORT,
      boost::asio::ip::address_v4::make_address_v4(UDP_MOCK_IP_ADDRESS.c_str()).to_uint(),
      UDP_MOCK_PORT) };
#else
  std::unique_ptr<communication_layer::UdpClientImpl> udp_client_{ new communication_layer::UdpClientImpl(
      std::bind(&UdpClientTests::handleNewData, this, _1, _2, _3),
      std::bind(&UdpClientTests::handleError, this, _1),
      HOST_UDP_PORT,
      boost::asio::ip::address_v4::from_string(UDP_MOCK_IP_ADDRESS.c_str()).to_ulong(),
      UDP_MOCK_PORT) };
#endif

  const data_conversion_layer::RawData send_array_{ 'H', 'e', 'l', 'l', 'o' };
  const udp::endpoint host_endpoint;
};

#if BOOST_VERSION > 107000
UdpClientTests::UdpClientTests()
  : host_endpoint(udp::endpoint(boost::asio::ip::make_address_v4(HOST_IP_ADDRESS.c_str()), HOST_UDP_PORT))
#elif BOOST_VERSION >= 106900
UdpClientTests::UdpClientTests()
  : host_endpoint(udp::endpoint(boost::asio::ip::address_v4::make_address_v4(HOST_IP_ADDRESS.c_str()), HOST_UDP_PORT))
#else
UdpClientTests::UdpClientTests()
  : host_endpoint(udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS.c_str()), HOST_UDP_PORT))
#endif
{
}

void UdpClientTests::sendTestDataToClient()
{
  mock_udp_server_.asyncSend(host_endpoint, send_array_);
}

void UdpClientTests::sendEmptyTestDataToClient()
{
  const psen_scan_v2_standalone::data_conversion_layer::RawData data;
  assert(data.empty());
  mock_udp_server_.asyncSend(host_endpoint, data);
}

data_conversion_layer::RawData createRawData(std::string dataString)
{
  data_conversion_layer::RawData write_buf;
  std::copy(dataString.begin(), dataString.end(), std::back_inserter(write_buf));
  return write_buf;
}

TEST_F(UdpClientTests, testGetHostIp)
{
  EXPECT_EQ(HOST_IP_ADDRESS, udp_client_->hostIp().to_string());
}

TEST_F(UdpClientTests, testAsyncReadOperation)
{
  util::Barrier client_received_data_barrier;
  EXPECT_CALL(*this, handleNewData(_, send_array_.size(), _)).WillOnce(OpenBarrier(&client_received_data_barrier));

  udp_client_->startAsyncReceiving();
  sendTestDataToClient();
  EXPECT_TRUE(client_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Udp client did not receive data";
}

TEST_F(UdpClientTests, testSingleAsyncReadOperation)
{
  util::Barrier client_received_data_barrier;
  EXPECT_CALL(*this, handleNewData(_, send_array_.size(), _)).WillOnce(OpenBarrier(&client_received_data_barrier));

  udp_client_->startAsyncReceiving(communication_layer::ReceiveMode::single);
  sendTestDataToClient();
  EXPECT_TRUE(client_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Udp client did not receive data";
}

TEST_F(UdpClientTests, Should_NotCallErrorCallback_WhenDestroyedWhileAsyncReceivePending)
{
  EXPECT_CALL(*this, handleError(_)).Times(0);

  udp_client_->startAsyncReceiving();
  udp_client_->close();
}

TEST_F(UdpClientTests, testErrorHandlingForReceive)
{
  util::Barrier error_callback_called_barrier;
  EXPECT_CALL(*this, handleError(_)).WillOnce(OpenBarrier(&error_callback_called_barrier));

  udp_client_->startAsyncReceiving(communication_layer::ReceiveMode::single);
  sendEmptyTestDataToClient();
  EXPECT_TRUE(error_callback_called_barrier.waitTillRelease(DEFAULT_TIMEOUT))
      << "Error callback should have been called";
}

TEST_F(UdpClientTests, testWriteOperation)
{
  auto write_buf = createRawData("Hello!");

  util::Barrier server_mock_received_data_barrier;
  EXPECT_CALL(*this, receivedUdpMsg(_, write_buf)).WillOnce(OpenBarrier(&server_mock_received_data_barrier));

  mock_udp_server_.asyncReceive();
  udp_client_->write(write_buf);

  EXPECT_TRUE(server_mock_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Server mock did not receive data";
}

TEST_F(UdpClientTests, testWritingWhileReceiving)
{
  auto write_buf = createRawData("Hello!");

  util::Barrier client_received_data_barrier;
  util::Barrier server_mock_received_data_barrier;
  EXPECT_CALL(*this, handleNewData(_, send_array_.size(), _)).WillOnce(OpenBarrier(&client_received_data_barrier));
  EXPECT_CALL(*this, receivedUdpMsg(_, write_buf))
      .WillOnce(DoAll(InvokeWithoutArgs(this, &UdpClientTests::sendTestDataToClient),
                      OpenBarrier(&server_mock_received_data_barrier)));

  mock_udp_server_.asyncReceive();

  udp_client_->startAsyncReceiving();

  udp_client_->write(write_buf);

  EXPECT_TRUE(server_mock_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Server mock did not receive data";
  EXPECT_TRUE(client_received_data_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Udp client did not receive data";
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
