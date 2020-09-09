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

#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/degree_to_rad.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
static constexpr int MINIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::min() };
static constexpr int MAXIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::max() };
static constexpr double MINIMAL_SCAN_ANGLE{ 0.0 };
static constexpr double MAXIMAL_SCAN_ANGLE{ degreeToRad(275.) };
static const double TOO_LARGE_SCAN_ANGLE{ std::nextafter(MAXIMAL_SCAN_ANGLE, MAXIMAL_SCAN_ANGLE + 1) };
static const double TOO_SMALL_SCAN_ANGLE{ std::nextafter(MINIMAL_SCAN_ANGLE, MINIMAL_SCAN_ANGLE - 1) };
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

  ScannerConfigurationBuilder& setStartAngle(const double& start_angle)
  {
    start_angle_ = start_angle;
    return *this;
  }

  ScannerConfigurationBuilder& setEndAngle(const double& end_angle)
  {
    end_angle_ = end_angle;
    return *this;
  }

  ScannerConfiguration build()
  {
    return ScannerConfiguration(
        host_ip_, host_udp_port_data_, host_udp_port_control_, client_ip_, start_angle_, end_angle_);
  }

protected:
  std::string host_ip_{ VALID_IP };
  std::string client_ip_{ VALID_IP };
  int host_udp_port_data_{ MAXIMAL_PORT_NUMBER - 1 };
  int host_udp_port_control_{ MAXIMAL_PORT_NUMBER };
  double start_angle_{ MINIMAL_SCAN_ANGLE };
  double end_angle_{ MAXIMAL_SCAN_ANGLE };
};

TEST_F(ScannerConfigurationTest, testConstructorSuccess)
{
  EXPECT_NO_THROW(ScannerConfigurationBuilder().build());
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidHostIp)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostIp(INVALID_IP).build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testConstructorInvalidClientIp)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setClientIp(INVALID_IP).build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testConstructorDataPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpDataPort(-1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorDataPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpDataPort(MAXIMAL_PORT_NUMBER + 1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorControlPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpControlPort(MINIMAL_PORT_NUMBER - 1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorControlPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setHostUdpControlPort(MAXIMAL_PORT_NUMBER + 1).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorStartAngleTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setStartAngle(TOO_SMALL_SCAN_ANGLE).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorStartAngleTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setStartAngle(TOO_LARGE_SCAN_ANGLE).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorEndAngleTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setEndAngle(TOO_SMALL_SCAN_ANGLE).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorEndAngleTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setEndAngle(TOO_LARGE_SCAN_ANGLE).build(), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, testConstructorEndAngleSmallerThanStartAngle)
{
  EXPECT_THROW(ScannerConfigurationBuilder().setStartAngle(2.0).setEndAngle(1.0).build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, testTargetIp)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  const auto host_ip = sc.hostIp();
  EXPECT_EQ(4U, sizeof(host_ip));

  // convert host_ip back to string representation
  const auto network_number = inet_makeaddr(host_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string host_ip_string(network_number_ascii);

  EXPECT_EQ(VALID_IP, host_ip_string);
}

TEST_F(ScannerConfigurationTest, testClientIp)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  const auto client_ip = sc.clientIp();
  EXPECT_EQ(4U, sizeof(client_ip));

  // convert host_ip back to string representation
  const auto network_number = inet_makeaddr(client_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string client_ip_string(network_number_ascii);

  EXPECT_EQ(VALID_IP, client_ip_string);
}

TEST_F(ScannerConfigurationTest, testUDPPorts)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  const auto host_udp_port_data = sc.hostUDPPortData();
  EXPECT_EQ(2U, sizeof(host_udp_port_data));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER - 1, static_cast<int>(host_udp_port_data));

  const auto host_udp_port_control = sc.hostUDPPortControl();
  EXPECT_EQ(2U, sizeof(host_udp_port_control));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER, static_cast<int>(host_udp_port_control));
}

TEST_F(ScannerConfigurationTest, testStartAngle)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  EXPECT_EQ(MINIMAL_SCAN_ANGLE, sc.startAngle());
}

TEST_F(ScannerConfigurationTest, testEndAngle)
{
  ScannerConfiguration sc{ ScannerConfigurationBuilder().build() };

  EXPECT_EQ(MAXIMAL_SCAN_ANGLE, sc.endAngle());
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
