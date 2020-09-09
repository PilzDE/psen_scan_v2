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
#include <array>
#include <functional>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/udp_client.h"

namespace psen_scan_v2_test
{
static constexpr unsigned short HOST_UDP_READ_PORT{ 45001 };
static constexpr unsigned short UDP_MOCK_SEND_PORT{ HOST_UDP_READ_PORT + 1 };

static const std::string UDP_MOCK_IP_ADDRESS{ "127.0.0.1" };

using std::placeholders::_1;
using std::placeholders::_2;

class CallbackHandler
{
public:
  MOCK_METHOD2(handleNewData, void(const psen_scan_v2::RawScannerData&, const std::size_t&));
  MOCK_METHOD1(handleError, void(const std::string&));
};

TEST(UdpClientTests, testInvalidNewDataHandler)
{
  CallbackHandler handler;

  EXPECT_THROW(psen_scan_v2::UdpClientImpl reader(nullptr,
                                                  std::bind(&CallbackHandler::handleError, &handler, _1),
                                                  HOST_UDP_READ_PORT,
                                                  inet_network(UDP_MOCK_IP_ADDRESS.c_str()),
                                                  UDP_MOCK_SEND_PORT),
               std::invalid_argument);
}

TEST(UdpClientTests, testInvalidErrorHandler)
{
  CallbackHandler handler;

  EXPECT_THROW(psen_scan_v2::UdpClientImpl reader(std::bind(&CallbackHandler::handleNewData, &handler, _1, _2),
                                                  nullptr,
                                                  HOST_UDP_READ_PORT,
                                                  inet_network(UDP_MOCK_IP_ADDRESS.c_str()),
                                                  UDP_MOCK_SEND_PORT),
               std::invalid_argument);
}

TEST(UdpClientTests, testCloseConnectionFailureForCompleteCoverage)
{
  std::unique_ptr<psen_scan_v2::CloseConnectionFailure> ex{ new psen_scan_v2::CloseConnectionFailure() };
}

TEST(UdpClientTests, testOpenConnectionFailureForCompleteCoverage)
{
  std::unique_ptr<psen_scan_v2::OpenConnectionFailure> ex{ new psen_scan_v2::OpenConnectionFailure() };
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
