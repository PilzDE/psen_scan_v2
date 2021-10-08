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

#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/start_request.h"
#include "psen_scan_v2_standalone/data_conversion_layer/start_request_serialization.h"
#include "psen_scan_v2_standalone/scan_range.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_test_helper.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
class StartRequestTest : public ::testing::Test
{
public:
  enum class Offset : std::size_t
  {
    crc = 0x00,
    seq_number = 0x04,
    reserved = 0x08,
    opcode = 0x10,
    ip = 0x14,
    udp_port = 0x18,
    device_enabled = 0x1A,
    intensities_enabled = 0x1B,
    point_in_safety_enabled = 0x1C,
    active_zone_set_enabled = 0x1D,
    io_pin_enabled = 0x1E,
    scan_counter_enabled = 0x1F,
    speed_encoder_enabled = 0x20,
    diagnostics_enabled = 0x21,
    master_start_angle = 0x22,
    master_end_angle = 0x24,
    master_angle_resolution = 0x26,
    slave_one_start_angle = 0x28,
    slave_one_end_angle = 0x2A,
    slave_one_angle_resolution = 0x2C,
    slave_two_start_angle = 0x2E,
    slave_two_end_angle = 0x30,
    slave_two_angle_resolution = 0x32,
    slave_three_start_angle = 0x34,
    slave_three_end_angle = 0x36,
    slave_three_angle_resolution = 0x38
  };
};

TEST_F(StartRequestTest, constructorTest)
{
  const std::string& host_ip = "192.168.0.1";
  const uint16_t& host_udp_port_data = 65535;

  const ScanRange scan_range{ util::TenthOfDegree(1), util::TenthOfDegree::fromRad(4.71) };

  uint32_t sequence_number{ 123 };
  data_conversion_layer::start_request::Message sr(ScannerConfigurationBuilder()
                                                       .hostIP(host_ip)
                                                       .hostDataPort(host_udp_port_data)
                                                       .hostControlPort(1 /* irrelevant */)
                                                       .scannerIp("192.168.0.50")
                                                       .scannerDataPort(77)
                                                       .scannerControlPort(78)
                                                       .scanRange(scan_range)
                                                       .scanResolution(util::TenthOfDegree(10))
                                                       .build());

  auto data = serialize(sr, sequence_number);
  boost::crc_32_type result;
  result.process_bytes(&data[sizeof(uint32_t)], data.size() - sizeof(uint32_t));

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::crc), (uint32_t)result.checksum()));

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::seq_number), (uint32_t)sequence_number));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::reserved), (uint64_t)0));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::opcode), (uint32_t)0x35));

#if BOOST_VERSION >= 107000
  EXPECT_TRUE(DecodingEquals(
      data, static_cast<size_t>(Offset::ip), boost::asio::ip::make_address_v4(host_ip.c_str()).to_uint(), Endian::big));
#elif BOOST_VERSION >= 106900
  EXPECT_TRUE(DecodingEquals(data,
                             static_cast<size_t>(Offset::ip),
                             boost::asio::ip::address_v4::make_address_v4(host_ip.c_str()).to_uint(),
                             Endian::big));
#else
  EXPECT_TRUE(DecodingEquals<uint32_t>(data,
                                       static_cast<size_t>(Offset::ip),
                                       boost::asio::ip::address_v4::from_string(host_ip.c_str()).to_ulong(),
                                       Endian::big));
#endif

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::udp_port), host_udp_port_data));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::device_enabled), (uint8_t)0b00001000));

  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::intensities_enabled), 0b00000000));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::point_in_safety_enabled), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::active_zone_set_enabled), 0b00001000));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::io_pin_enabled), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::scan_counter_enabled), 0b00001000));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::speed_encoder_enabled), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::diagnostics_enabled), 0));

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::master_start_angle), scan_range.getStart().value()));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::master_end_angle), scan_range.getEnd().value()));
  EXPECT_TRUE(DecodingEquals(
      data, static_cast<size_t>(Offset::master_angle_resolution), data_conversion_layer::degreeToTenthDegree(1.0)));

  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_one_start_angle), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_one_end_angle), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_one_angle_resolution), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_two_start_angle), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_two_end_angle), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_two_angle_resolution), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_three_start_angle), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_three_end_angle), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::slave_three_angle_resolution), 0));
}

TEST_F(StartRequestTest, endAngleIncreasedWhenMatchingDataPoint)
{
  const ScanRange scan_range{ util::TenthOfDegree(1u), util::TenthOfDegree(2749u) };
  const util::TenthOfDegree resolution{ 2u };

  ScannerConfigurationBuilder builder;
  builder.hostIP("192.168.0.50").scannerIp("192.168.0.10").scanResolution(resolution).scanRange(scan_range);

  const ScannerConfiguration config = builder.build();

  const auto raw_start_request{ data_conversion_layer::start_request::serialize(
      data_conversion_layer::start_request::Message(config)) };

  EXPECT_TRUE(DecodingEquals(
      raw_start_request, static_cast<size_t>(Offset::master_start_angle), scan_range.getStart().value()));
  EXPECT_TRUE(DecodingEquals<uint16_t>(
      raw_start_request, static_cast<size_t>(Offset::master_end_angle), scan_range.getEnd().value() + 1));
  EXPECT_TRUE(
      DecodingEquals(raw_start_request, static_cast<size_t>(Offset::master_angle_resolution), resolution.value()));
}

TEST_F(StartRequestTest, crcWithIntensities)
{
  ScannerConfigurationBuilder builder;
  builder.hostIP("192.168.0.50")
      .scannerIp("192.168.0.10")
      .scanResolution(util::TenthOfDegree(2u))
      .enableIntensities()
      .scanRange(ScanRange(util::TenthOfDegree(1), util::TenthOfDegree(2749)))
      .enableDiagnostics();

  const ScannerConfiguration config = builder.build();

  const auto raw_start_request{ data_conversion_layer::start_request::serialize(
      data_conversion_layer::start_request::Message(config)) };

  // see wireshark for this number
  // generated with `roslaunch psen_scan_v2 psen_scan_v2.launch intensities:=true resolution:=0.0035`
  const std::array<unsigned char, 4> expected_crc = { 0x9d, 0xfb, 0x2c, 0x7a };

  for (size_t i = 0; i < expected_crc.size(); ++i)
  {
    EXPECT_EQ(static_cast<unsigned int>(static_cast<unsigned char>(raw_start_request[i])), expected_crc[i]);
  }
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
