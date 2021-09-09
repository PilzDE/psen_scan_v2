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

#include <array>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/communication_layer/udp_client.h"

namespace psen_scan_v2_standalone_test
{
static constexpr unsigned short HOST_UDP_READ_PORT{ 45001 };
static constexpr unsigned short UDP_MOCK_SEND_PORT{ HOST_UDP_READ_PORT + 1 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class CallbackHandler
{
public:
  MOCK_METHOD3(handleNewData,
               void(const psen_scan_v2_standalone::data_conversion_layer::RawDataConstPtr&,
                    const std::size_t&,
                    const int64_t&));
  MOCK_METHOD1(handleError, void(const std::string&));
};

TEST(UdpClientTests, testInvalidNewDataHandler)
{
  CallbackHandler handler;

#if BOOST_VERSION > 107000
  EXPECT_THROW(psen_scan_v2_standalone::communication_layer::UdpClientImpl reader(
                   nullptr,
                   std::bind(&CallbackHandler::handleError, &handler, _1),
                   HOST_UDP_READ_PORT,
                   boost::asio::ip::make_address_v4(UDP_MOCK_IP_ADDRESS.c_str()).to_uint(),
                   UDP_MOCK_SEND_PORT),
               std::invalid_argument);
#elif BOOST_VERSION >= 106900
  EXPECT_THROW(psen_scan_v2_standalone::communication_layer::UdpClientImpl reader(
                   nullptr,
                   std::bind(&CallbackHandler::handleError, &handler, _1),
                   HOST_UDP_READ_PORT,
                   boost::asio::ip::address_v4::make_address_v4(UDP_MOCK_IP_ADDRESS.c_str()).to_uint(),
                   UDP_MOCK_SEND_PORT),
               std::invalid_argument);
#else
  EXPECT_THROW(psen_scan_v2_standalone::communication_layer::UdpClientImpl reader(
                   nullptr,
                   std::bind(&CallbackHandler::handleError, &handler, _1),
                   HOST_UDP_READ_PORT,
                   boost::asio::ip::address_v4::from_string(UDP_MOCK_IP_ADDRESS.c_str()).to_ulong(),
                   UDP_MOCK_SEND_PORT),
               std::invalid_argument);
#endif
}

TEST(UdpClientTests, testInvalidErrorHandler)
{
  CallbackHandler handler;
#if BOOST_VERSION > 107000
  EXPECT_THROW(psen_scan_v2_standalone::communication_layer::UdpClientImpl reader(
                   std::bind(&CallbackHandler::handleNewData, &handler, _1, _2, _3),
                   nullptr,
                   HOST_UDP_READ_PORT,
                   boost::asio::ip::make_address_v4(UDP_MOCK_IP_ADDRESS.c_str()).to_uint(),
                   UDP_MOCK_SEND_PORT),
               std::invalid_argument);
#elif BOOST_VERSION >= 106900
  EXPECT_THROW(psen_scan_v2_standalone::communication_layer::UdpClientImpl reader(
                   std::bind(&CallbackHandler::handleNewData, &handler, _1, _2, _3),
                   nullptr,
                   HOST_UDP_READ_PORT,
                   boost::asio::ip::address_v4::make_address_v4(UDP_MOCK_IP_ADDRESS.c_str()).to_uint(),
                   UDP_MOCK_SEND_PORT),
               std::invalid_argument);
#else
  EXPECT_THROW(psen_scan_v2_standalone::communication_layer::UdpClientImpl reader(
                   std::bind(&CallbackHandler::handleNewData, &handler, _1, _2, _3),
                   nullptr,
                   HOST_UDP_READ_PORT,
                   boost::asio::ip::address_v4::from_string(UDP_MOCK_IP_ADDRESS.c_str()).to_ulong(),
                   UDP_MOCK_SEND_PORT),
               std::invalid_argument);
#endif
}

TEST(UdpClientTests, testCloseConnectionFailureForCompleteCoverage)
{
  std::unique_ptr<psen_scan_v2_standalone::communication_layer::UdpClientImpl::CloseConnectionFailure> ex{
    new psen_scan_v2_standalone::communication_layer::UdpClientImpl::CloseConnectionFailure()
  };
}

TEST(UdpClientTests, testOpenConnectionFailureForCompleteCoverage)
{
  std::unique_ptr<psen_scan_v2_standalone::communication_layer::UdpClientImpl::OpenConnectionFailure> ex{
    new psen_scan_v2_standalone::communication_layer::UdpClientImpl::OpenConnectionFailure()
  };
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
