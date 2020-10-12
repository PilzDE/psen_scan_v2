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

#include <stdexcept>
#include <string>
#include <limits>

#include <arpa/inet.h>
#include <gtest/gtest.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scan_range.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
static constexpr int MINIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::min() };
static constexpr int MAXIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::max() };
static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(2750) };
static const std::string VALID_IP{ "127.0.0.1" };
static const std::string INVALID_IP{ "invalid_ip" };

class ScannerConfigurationTest : public testing::Test
{
};

class ScannerConfigurationBuilder
{
public:
  ScannerConfigurationBuilder& setHostIp(const std::string& host_ip)
  {
    host_ip_ = host_ip;
    return *this;
  }

  ScannerConfigurationBuilder& setClientIp(const std::string& client_ip)
  {
    client_ip_ = client_ip;
    return *this;
  }

  ScannerConfigurationBuilder& setHostUdpDataPort(const int& host_udp_port_data)
  {
    host_udp_port_data_ = host_udp_port_data;
    return *this;
  }

  ScannerConfigurationBuilder& setHostUdpControlPort(const int& host_udp_port_control)
  {
    host_udp_port_control_ = host_udp_port_control;
    return *this;
  }

  ScannerConfiguration build()
  {
    return ScannerConfiguration(host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, scan_range_);
  }

protected:
  std::string host_ip_{ VALID_IP };
  std::string client_ip_{ VALID_IP };
  int host_udp_port_data_{ MAXIMAL_PORT_NUMBER - 1 };
  int host_udp_port_control_{ MAXIMAL_PORT_NUMBER };
  const DefaultScanRange scan_range_{ SCAN_RANGE };
};

TEST_F(ScannerConfigurationTest, constructorSuccess)
{
  EXPECT_NO_THROW(ScannerConfigurationBuilder().build());
}

TEST_F(ScannerConfigurationTest, constructorInvalidHostIp)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostIp(INVALID_IP).build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, constructorInvalidClientIp)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setClientIp(INVALID_IP).build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, constructorDataPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpDataPort(-1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, constructorDataPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpDataPort(MAXIMAL_PORT_NUMBER + 1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, constructorControlPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpControlPort(MINIMAL_PORT_NUMBER - 1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, constructorControlPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpControlPort(MAXIMAL_PORT_NUMBER + 1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, targetIp)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  const auto host_ip = sc.hostIp();
  EXPECT_EQ(4U, sizeof(host_ip));

  const auto network_number = inet_makeaddr(host_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string host_ip_string(network_number_ascii);

  EXPECT_EQ(VALID_IP, host_ip_string);
}

TEST_F(ScannerConfigurationTest, clientIp)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  const auto client_ip = sc.clientIp();
  EXPECT_EQ(4U, sizeof(client_ip));

  const auto network_number = inet_makeaddr(client_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string client_ip_string(network_number_ascii);

  EXPECT_EQ(VALID_IP, client_ip_string);
}

TEST_F(ScannerConfigurationTest, udpPorts)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  const auto host_udp_port_data = sc.hostUDPPortData();
  EXPECT_EQ(2U, sizeof(host_udp_port_data));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER - 1, static_cast<int>(host_udp_port_data));

  const auto host_udp_port_control = sc.hostUDPPortControl();
  EXPECT_EQ(2U, sizeof(host_udp_port_control));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER, static_cast<int>(host_udp_port_control));
}

TEST_F(ScannerConfigurationTest, startAngle)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  EXPECT_EQ(SCAN_RANGE.getStart(), sc.scanRange().getStart());
}

TEST_F(ScannerConfigurationTest, endAngle)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  EXPECT_EQ(SCAN_RANGE.getEnd(), sc.scanRange().getEnd());
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
