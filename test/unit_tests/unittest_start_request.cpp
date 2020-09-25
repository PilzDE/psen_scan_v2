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

#include <arpa/inet.h>
#include <boost/crc.hpp>
#include <gtest/gtest.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/raw_data_test_helper.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
class StartRequestTest : public ::testing::Test
{
};

TEST_F(StartRequestTest, constructorTest)
{
  const std::string& host_ip = "192.168.0.1";
  const uint16_t& host_udp_port_data = 65535;

  const double start_angle{ 0.0 };
  const double end_angle{ 4.71 };

  ScannerConfiguration sc(host_ip, host_udp_port_data, 0 /* irrelevant */, "192.168.0.50", start_angle, end_angle);

  uint32_t sequence_number{ 123 };
  StartRequest sr(sc, sequence_number);

  auto data = sr.toRawData();
  boost::crc_32_type result;
  result.process_bytes(&data[sizeof(uint32_t)], data.size() - sizeof(uint32_t));

  EXPECT_TRUE(DecodingEquals(data, 0x00, (uint32_t)result.checksum()));  // CRC

  EXPECT_TRUE(DecodingEquals(data, 0x00, 0xd6224cb9));  // CRC - Fixed for now, Note: Other byte order as in wireshark

  EXPECT_TRUE(DecodingEquals(data, 0x04, (uint32_t)sequence_number));  // SequenceNumber

  EXPECT_TRUE(DecodingEquals(data, 0x08, (uint64_t)0));  // Reserved

  EXPECT_TRUE(DecodingEquals(data, 0x10, (uint32_t)0x35));  // OP code

  EXPECT_TRUE(DecodingEquals(data, 0x14, inet_network(host_ip.c_str()), Endian::BIG));  // IP

  EXPECT_TRUE(DecodingEquals(data, 0x18, host_udp_port_data));  // UDP port

  EXPECT_TRUE(DecodingEquals(data, 0x1A, (uint8_t)0b00001000));  // Device enabled

  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x1B, 0));           // Intensities enabled
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x1C, 0));           // Point in safety enabled
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x1D, 0));           // Active zone set enabled
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x1E, 0));           // IO Pin enabled
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x1F, 0b00001000));  // Scan counter enabled
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x20, 0));           // Speed encoder enabled
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, 0x21, 0));           // Diagnostics enabled

  EXPECT_TRUE(DecodingEquals(data, 0x22, radToTenthDegree(start_angle)));          // Master Start Angle
  EXPECT_TRUE(DecodingEquals(data, 0x24, radToTenthDegree(end_angle)));            // Master End Angle
  EXPECT_TRUE(DecodingEquals(data, 0x26, degreeToTenthDegree(0.1)));  // Master Angle Resolution

  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x28, 0));  // Slave 1 Start Angle
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x2A, 0));  // Slave 1 End Angle
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x2C, 0));  // Slave 1 Angle Resolution

  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x2E, 0));  // Slave 2 Start Angle
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x30, 0));  // Slave 2 End Angle
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x32, 0));  // Slave 2 Angle Resolution

  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x34, 0));  // Slave 3 Start Angle
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x36, 0));  // Slave 3 End Angle
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, 0x38, 0));  // Slave 3 Angle Resolution
}

TEST_F(StartRequestTest, regressionForRealSystem)
{
  ScannerConfiguration sc("192.168.0.50", 55115, 0, "192.168.0.10", 0.0, degreeToRadian(275.));
  StartRequest sr(sc, 0);

  auto data = sr.toRawData();

  unsigned char expected_crc[4] = { 0xed, 0xc3, 0x48, 0xa1 };  // see wireshark for this number

  for (size_t i = 0; i < sizeof(expected_crc); i++)
  {
    EXPECT_EQ(static_cast<unsigned int>(static_cast<unsigned char>(data[i])), expected_crc[i]);
  }
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
