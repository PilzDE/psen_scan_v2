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

#ifndef PSEN_SCAN_V2_TEST_MOCK_UDP_CLIENT_H
#define PSEN_SCAN_V2_TEST_MOCK_UDP_CLIENT_H

#include <gmock/gmock.h>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/udp_client.h"

namespace psen_scan_v2_test
{
class MockUdpClient
{
public:
  MockUdpClient(const psen_scan_v2::NewDataHandler& data_handler,
                const psen_scan_v2::ErrorHandler& error_handler,
                const unsigned short& host_port,
                const unsigned int& endpoint_ip,
                const unsigned short& endpoint_port){};

public:
  MOCK_METHOD0(close, void());
  MOCK_METHOD1(startReceiving, void(const std::chrono::high_resolution_clock::duration timeout));
  MOCK_METHOD1(write, void(const psen_scan_v2::DynamicSizeRawData& data));
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_MOCK_UDP_CLIENT_H
