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

#include <boost/asio.hpp>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/util/ip_conversion.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/scan_range.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static constexpr int MINIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::min() };
static constexpr int MAXIMAL_PORT_NUMBER{ std::numeric_limits<uint16_t>::max() };
static constexpr ScanRange SCAN_RANGE{ util::TenthOfDegree(1), util::TenthOfDegree(2749) };
static constexpr util::TenthOfDegree SCAN_RESOLUTION{ 2u };
static const std::string VALID_IP{ "127.0.0.1" };
static const std::string VALID_IP_OTHER{ "192.168.0.1" };
static const std::string INVALID_IP{ "invalid_ip" };
static const std::string EMPTY_IP{ "" };
static const std::string AUTO_IP{ "auto" };

class ScannerConfigurationTest : public testing::Test
{
};

static ScannerConfiguration createValidConfig()
{
  return ScannerConfigurationBuilder(VALID_IP)
      .hostIP(VALID_IP)
      .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
      .hostControlPort(MAXIMAL_PORT_NUMBER)
      .scannerDataPort(MINIMAL_PORT_NUMBER + 1)
      .scannerControlPort(MINIMAL_PORT_NUMBER + 2)
      .scanRange(SCAN_RANGE)
      .scanResolution(SCAN_RESOLUTION)
      .enableDiagnostics()
      .enableIntensities()
      .build();
}

static ScannerConfiguration createValidConfig(const std::string& host_ip)
{
  return ScannerConfigurationBuilder(VALID_IP)
      .hostIP(host_ip)
      .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
      .hostControlPort(MAXIMAL_PORT_NUMBER)
      .scannerDataPort(MINIMAL_PORT_NUMBER + 1)
      .scannerControlPort(MINIMAL_PORT_NUMBER + 2)
      .scanRange(SCAN_RANGE)
      .scanResolution(SCAN_RESOLUTION)
      .enableDiagnostics()
      .enableIntensities()
      .enableFragmentedScans(true)
      .build();
}

static ScannerConfiguration createValidDefaultConfig()
{
  return ScannerConfigurationBuilder(VALID_IP).scanRange(SCAN_RANGE);
}

TEST_F(ScannerConfigurationTest, shouldNotThrowInCaseOfValidConfiguration)
{
  EXPECT_NO_THROW(createValidConfig());
  EXPECT_NO_THROW(createValidDefaultConfig());
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScanRangeMissing)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP)
                   .hostIP(VALID_IP)
                   .hostDataPort(MAXIMAL_PORT_NUMBER - 1)
                   .hostControlPort(MAXIMAL_PORT_NUMBER)
                   .scannerDataPort(MINIMAL_PORT_NUMBER)
                   .scannerControlPort(MINIMAL_PORT_NUMBER + 1)
                   .build(),
               std::runtime_error);
}

TEST_F(ScannerConfigurationTest, shouldThrowInCaseOfInvalidHostIp)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).hostIP(INVALID_IP), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, shouldNotThrowInCaseOfEmptyHostIp)
{
  EXPECT_NO_THROW(ScannerConfigurationBuilder(VALID_IP).hostIP(EMPTY_IP));
}

TEST_F(ScannerConfigurationTest, shouldNotThrowInCaseOfAutoHostIp)
{
  EXPECT_NO_THROW(ScannerConfigurationBuilder(VALID_IP).hostIP(AUTO_IP));
}

TEST_F(ScannerConfigurationTest, shouldThrowInCaseOfInvalidScannerIp)
{
  EXPECT_THROW(ScannerConfigurationBuilder(INVALID_IP).build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostDataPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).hostDataPort(-1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostDataPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).hostDataPort(MAXIMAL_PORT_NUMBER + 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostControlPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).hostControlPort(MINIMAL_PORT_NUMBER - 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfHostControlPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).hostControlPort(MAXIMAL_PORT_NUMBER + 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerDataPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).scannerDataPort(-1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerDataPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).scannerDataPort(MAXIMAL_PORT_NUMBER + 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerControlPortTooSmall)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).scannerControlPort(MINIMAL_PORT_NUMBER - 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldThrowIfScannerControlPortTooLarge)
{
  EXPECT_THROW(ScannerConfigurationBuilder(VALID_IP).scannerControlPort(MAXIMAL_PORT_NUMBER + 1), std::out_of_range);
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectPortsAfterDefaultConstruction)
{
  const ScannerConfiguration config{ createValidDefaultConfig() };

  EXPECT_EQ(configuration::DATA_PORT_OF_SCANNER_DEVICE, config.scannerDataPort());
  EXPECT_EQ(configuration::CONTROL_PORT_OF_SCANNER_DEVICE, config.scannerControlPort());
  EXPECT_EQ(configuration::DATA_PORT_OF_HOST_DEVICE, config.hostUDPPortData());
  EXPECT_EQ(configuration::CONTROL_PORT_OF_HOST_DEVICE, config.hostUDPPortControl());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectDiagnosticsFlagAfterDefaultConstruction)
{
  const ScannerConfiguration config{ createValidDefaultConfig() };

  EXPECT_FALSE(config.diagnosticsEnabled());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectHostIpAfterConstruction)
{
  ScannerConfiguration sc{ createValidConfig() };

  const auto host_ip = *sc.hostIp();
  EXPECT_EQ(4U, sizeof(host_ip));

  const std::string host_ip_string(boost::asio::ip::address_v4(host_ip).to_string());

  EXPECT_EQ(VALID_IP, host_ip_string);
}

TEST_F(ScannerConfigurationTest, shouldReturnNoHostIpByDefault)
{
  ScannerConfiguration sc{ createValidDefaultConfig() };
  EXPECT_EQ(boost::none, sc.hostIp());
}

TEST_F(ScannerConfigurationTest, shouldReturnNoHostIpIfSetEmpty)
{
  ScannerConfiguration sc{ createValidConfig(EMPTY_IP) };
  EXPECT_EQ(boost::none, sc.hostIp());
}

TEST_F(ScannerConfigurationTest, shouldReturnNoHostIpIfSetAuto)
{
  ScannerConfiguration sc{ createValidConfig(AUTO_IP) };
  EXPECT_EQ(boost::none, sc.hostIp());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectScannerIpAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };

  const auto client_ip = sc.clientIp();
  EXPECT_EQ(4U, sizeof(client_ip));

  const std::string client_ip_string(boost::asio::ip::address_v4(client_ip).to_string());

  EXPECT_EQ(VALID_IP, client_ip_string);
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectHostPortsAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };

  const auto host_udp_port_data = sc.hostUDPPortData();
  EXPECT_EQ(2U, sizeof(host_udp_port_data));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER - 1, static_cast<int>(host_udp_port_data));

  const auto host_udp_port_control = sc.hostUDPPortControl();
  EXPECT_EQ(2U, sizeof(host_udp_port_control));
  EXPECT_EQ(MAXIMAL_PORT_NUMBER, static_cast<int>(host_udp_port_control));
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectScannerPortsAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };

  const auto scanner_udp_port_data = sc.scannerDataPort();
  EXPECT_EQ(2U, sizeof(scanner_udp_port_data));
  EXPECT_EQ(MINIMAL_PORT_NUMBER + 1, static_cast<int>(scanner_udp_port_data));

  const auto scanner_udp_port_control = sc.scannerControlPort();
  EXPECT_EQ(2U, sizeof(scanner_udp_port_control));
  EXPECT_EQ(MINIMAL_PORT_NUMBER + 2, static_cast<int>(scanner_udp_port_control));
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectStartAngleAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };

  EXPECT_EQ(SCAN_RANGE.getStart(), sc.scanRange().getStart());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectEndAngleAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };

  EXPECT_EQ(SCAN_RANGE.getEnd(), sc.scanRange().getEnd());
}

TEST_F(ScannerConfigurationTest, shouldReturnSetHostIp)
{
  ScannerConfiguration sc{ createValidConfig() };

  const uint32_t host_ip{ util::convertIP(VALID_IP_OTHER) };
  sc.hostIp(host_ip);

  EXPECT_EQ(host_ip, sc.hostIp());
}

TEST_F(ScannerConfigurationTest, shouldReturnCorrectResolutionAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };
  EXPECT_EQ(SCAN_RESOLUTION, sc.scanResolution());
}

TEST_F(ScannerConfigurationTest, shouldHaveCorrectResolutionOnDefault)
{
  const ScannerConfiguration sc{ createValidDefaultConfig() };
  EXPECT_EQ(psen_scan_v2_standalone::util::TenthOfDegree::fromRad(configuration::DEFAULT_SCAN_ANGLE_RESOLUTION),
            sc.scanResolution());
}

TEST_F(ScannerConfigurationTest, shouldHaveEnabledIntensitiesAfterConstruction)
{
  const ScannerConfiguration sc{ createValidConfig() };
  EXPECT_TRUE(sc.intensitiesEnabled());
}

TEST_F(ScannerConfigurationTest, shouldLoadIntensitiesFromConfigByDefault)
{
  const ScannerConfiguration sc{ createValidDefaultConfig() };
  EXPECT_EQ(configuration::INTENSITIES, sc.intensitiesEnabled());
}

TEST_F(ScannerConfigurationTest, shouldThrowInvalidArgumentWithLowResolutionAndEnabledIntensitiesOnBuild)
{
  auto sb = ScannerConfigurationBuilder(VALID_IP)
                .scanRange(SCAN_RANGE)
                .enableIntensities()
                .scanResolution(util::TenthOfDegree{ 1u });
  EXPECT_THROW(sb.build(), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, shouldThrowInvalidArgumentWithResolutionViolatingLowerLimit)
{
  auto sb = ScannerConfigurationBuilder(VALID_IP).scanRange(SCAN_RANGE);
  EXPECT_THROW(sb.scanResolution(util::TenthOfDegree{ 0u }), std::invalid_argument);
}

TEST_F(ScannerConfigurationTest, shouldThrowInvalidArgumentWithResolutionViolatingUpperLimit)
{
  auto sb = ScannerConfigurationBuilder(VALID_IP).scanRange(SCAN_RANGE);
  EXPECT_THROW(sb.scanResolution(util::TenthOfDegree{ 101u }), std::invalid_argument);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
