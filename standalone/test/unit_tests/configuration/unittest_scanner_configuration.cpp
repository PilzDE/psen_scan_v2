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

#include <stdexcept>
#include <string>
#include <limits>

#include <arpa/inet.h>
#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/configuration/scanner_configuration.h"
#include "psen_scan_v2_standalone/configuration/scanner_config_builder.h"
#include "psen_scan_v2_standalone/configuration/scan_range.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static constexpr int MINIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::min() };
static constexpr int MAXIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::max() };
static constexpr configuration::DefaultScanRange SCAN_RANGE{ util::TenthOfDegree(0), util::TenthOfDegree(2750) };
static const std::string VALID_IP{ "127.0.0.1" };
static const std::string INVALID_IP{ "invalid_ip" };

class ScannerConfigurationTest : public testing::Test
{
};

static configuration::ScannerConfiguration createValidConfig()
{
  return configuration::ScannerConfigurationBuilder()
      .hostIP(VALID_IP)
      .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
      .hostControlPort(MAXIMAL_PORT_NUMBER)
      .scannerIp(VALID_IP)
      .scannerDataPort(MINIMAL_PORT_NUMBER + 1)
      .scannerControlPort(MINIMAL_PORT_NUMBER + 2)
      .scanRange(SCAN_RANGE)
      .enableDiagnostics()
      .build();
}

static configuration::ScannerConfiguration createValidDefaultConfig()
{
  return configuration::ScannerConfigurationBuilder()
      .hostIP(VALID_IP)
      .scannerIp(VALID_IP)
      .scanRange(SCAN_RANGE)
      .build();
}

TEST_F(ScannerConfigurationTest, shouldNotThrowInCaseOfValidConfiguration)
{
  EXPECT_NO_THROW(createValidConfig());
  EXPECT_NO_THROW(createValidDefaultConfig());
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostIPMissing)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder()
                   .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
                   .hostControlPort(MAXIMAL_PORT_NUMBER)
                   .scannerIp(VALID_IP)
                   .scannerDataPort(MINIMAL_PORT_NUMBER)
                   .scannerControlPort(MINIMAL_PORT_NUMBER + 1)
                   .scanRange(SCAN_RANGE)
                   .build(),
               std::runtime_error);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerIPMissing)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder()
                   .hostIP(VALID_IP)
                   .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
                   .hostControlPort(MAXIMAL_PORT_NUMBER)
                   .scannerDataPort(MINIMAL_PORT_NUMBER)
                   .scannerControlPort(MINIMAL_PORT_NUMBER + 1)
                   .scanRange(SCAN_RANGE)
                   .build(),
               std::runtime_error);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScanRangeMissing)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder()
                   .hostIP(VALID_IP)
                   .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
                   .hostControlPort(MAXIMAL_PORT_NUMBER)
                   .scannerIp(VALID_IP)
                   .scannerDataPort(MINIMAL_PORT_NUMBER)
                   .scannerControlPort(MINIMAL_PORT_NUMBER + 1)
                   .build(),
               std::runtime_error);
}

TEST_F(ScannerConfigurationTest, shouldThrowInCaseOfInvalidHostIp)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().hostIP(INVALID_IP), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, shouldThrowInCaseOfInvalidScannerIp)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().scannerIp(INVALID_IP), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostDataPortTooSmall)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().hostDataPort(-1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostDataPortTooLarge)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().hostDataPort(MAXIMAL_PORT_NUMBER + 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostControlPortTooSmall)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().hostControlPort(MINIMAL_PORT_NUMBER - 1),
               std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostControlPortTooLarge)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().hostControlPort(MAXIMAL_PORT_NUMBER + 1),
               std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerDataPortTooSmall)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().scannerDataPort(-1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerDataPortTooLarge)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().scannerDataPort(MAXIMAL_PORT_NUMBER + 1),
               std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerControlPortTooSmall)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().scannerControlPort(MINIMAL_PORT_NUMBER - 1),
               std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerControlPortTooLarge)
{
  EXPECT_THROW(configuration::ScannerConfigurationBuilder().scannerControlPort(MAXIMAL_PORT_NUMBER + 1),
               std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectPortsAfterDefaultConstruction)
{
  const configuration::ScannerConfiguration config{ createValidDefaultConfig() };

  EXPECT_EQ(configuration::DATA_PORT_OF_SCANNER_DEVICE, config.scannerDataPort());
  EXPECT_EQ(configuration::CONTROL_PORT_OF_SCANNER_DEVICE, config.scannerControlPort());
  EXPECT_EQ(configuration::DATA_PORT_OF_HOST_DEVICE, config.hostUDPPortData());
  EXPECT_EQ(configuration::CONTROL_PORT_OF_HOST_DEVICE, config.hostUDPPortControl());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectDiagnosticsFlagAfterDefaultConstruction)
{
  const configuration::ScannerConfiguration config{ createValidDefaultConfig() };

  EXPECT_FALSE(config.diagnosticsEnabled());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectHostIpAfterConstruction)
{
  configuration::ScannerConfiguration sc{ createValidConfig() };

  const auto host_ip = sc.hostIp();
  EXPECT_EQ(4U, sizeof(host_ip));

  const auto network_number = inet_makeaddr(host_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string host_ip_string(network_number_ascii);

  EXPECT_EQ(VALID_IP, host_ip_string);
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectScannerIpAfterConstruction)
{
  const configuration::ScannerConfiguration sc{ createValidConfig() };

  const auto client_ip = sc.clientIp();
  EXPECT_EQ(4U, sizeof(client_ip));

  const auto network_number = inet_makeaddr(client_ip, 0);
  const auto network_number_ascii = inet_ntoa(network_number);
  const std::string client_ip_string(network_number_ascii);

  EXPECT_EQ(VALID_IP, client_ip_string);
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectHostPortsAfterConstruction)
{
  const configuration::ScannerConfiguration sc{ createValidConfig() };

  const auto host_udp_port_data = sc.hostUDPPortData();
  EXPECT_EQ(2U, sizeof(host_udp_port_data));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER - 1, static_cast<int>(host_udp_port_data));

  const auto host_udp_port_control = sc.hostUDPPortControl();
  EXPECT_EQ(2U, sizeof(host_udp_port_control));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER, static_cast<int>(host_udp_port_control));
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectScannerPortsAfterConstruction)
{
  const configuration::ScannerConfiguration sc{ createValidConfig() };

  const auto scanner_udp_port_data = sc.scannerDataPort();
  EXPECT_EQ(2U, sizeof(scanner_udp_port_data));
  EXPECT_EQ(MINIMAL_PORT_NUMBER + 1, static_cast<int>(scanner_udp_port_data));

  const auto scanner_udp_port_control = sc.scannerControlPort();
  EXPECT_EQ(2U, sizeof(scanner_udp_port_control));
  EXPECT_EQ(MINIMAL_PORT_NUMBER + 2, static_cast<int>(scanner_udp_port_control));
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectStartAngleAfterConstruction)
{
  const configuration::ScannerConfiguration sc{ createValidConfig() };

  EXPECT_EQ(SCAN_RANGE.getStart(), sc.scanRange().getStart());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectEndAngleAfterConstruction)
{
  const configuration::ScannerConfiguration sc{ createValidConfig() };

  EXPECT_EQ(SCAN_RANGE.getEnd(), sc.scanRange().getEnd());
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
