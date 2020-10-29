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
#include "psen_scan_v2/scan_range.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
class StartRequestTest : public ::testing::Test
{
public:
  enum class Offset : std::size_t
  {
    CRC = 0x00,
    SEQ_NUMBER = 0x04,
    RESERVED = 0x08,
    OPCODE = 0x10,
    IP = 0x14,
    UDP_PORT = 0x18,
    DEVICE_ENABLED = 0x1A,
    INTENSITIES_ENABLED = 0x1B,
    POINT_IN_SAFETY_ENABLED = 0x1C,
    ACTIVE_ZONE_SET_ENABLED = 0x1D,
    IO_PIN_ENABLED = 0x1E,
    SCAN_COUNTER_ENABLED = 0x1F,
    SPEED_ENCODER_ENABLED = 0x20,
    DIAGNOSTICS_ENABLED = 0x21,
    MASTER_START_ANGLE = 0x22,
    MASTER_END_ANGLE = 0x24,
    MASTER_ANGLE_RESOLUTION = 0x26,
    SLAVE_ONE_START_ANGLE = 0x28,
    SLAVE_ONE_END_ANGLE = 0x2A,
    SLAVE_ONE_ANGLE_RESOLUTION = 0x2C,
    SLAVE_TWO_START_ANGLE = 0x2E,
    SLAVE_TWO_END_ANGLE = 0x30,
    SLAVE_TWO_ANGLE_RESOLUTION = 0x32,
    SLAVE_THREE_START_ANGLE = 0x34,
    SLAVE_THREE_END_ANGLE = 0x36,
    SLAVE_THREE_ANGLE_RESOLUTION = 0x38
  };
};

TEST_F(StartRequestTest, constructorTest)
{
  const std::string& host_ip = "192.168.0.1";
  const uint16_t& host_udp_port_data = 65535;

  const DefaultScanRange scan_range{ TenthOfDegree(0), TenthOfDegree::fromRad(4.71) };

  ScannerConfiguration sc(host_ip, host_udp_port_data, 0 /* irrelevant */, "192.168.0.50", scan_range, false);

  uint32_t sequence_number{ 123 };
  StartRequest sr(sc, sequence_number);

  auto data = sr.serialize();
  boost::crc_32_type result;
  result.process_bytes(&data[sizeof(uint32_t)], data.size() - sizeof(uint32_t));

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::CRC), (uint32_t)result.checksum()));

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::CRC), 0xeb447fb));  // CRC - Fixed for now, Note:
                                                                                   // Other byte order as in
                                                                                   // wireshark

  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::SEQ_NUMBER), (uint32_t)sequence_number));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::RESERVED), (uint64_t)0));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::OPCODE), (uint32_t)0x35));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::IP), inet_network(host_ip.c_str()), Endian::BIG));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::UDP_PORT), host_udp_port_data));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::DEVICE_ENABLED), (uint8_t)0b00001000));

  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::INTENSITIES_ENABLED), 0b00001000));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::POINT_IN_SAFETY_ENABLED), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::ACTIVE_ZONE_SET_ENABLED), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::IO_PIN_ENABLED), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::SCAN_COUNTER_ENABLED), 0b00001000));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::SPEED_ENCODER_ENABLED), 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, static_cast<size_t>(Offset::DIAGNOSTICS_ENABLED), 0));

  EXPECT_TRUE(
      DecodingEquals(data, static_cast<size_t>(Offset::MASTER_START_ANGLE), static_cast<uint16_t>(scan_range.getStart().value())));
  EXPECT_TRUE(
      DecodingEquals(data, static_cast<size_t>(Offset::MASTER_END_ANGLE), static_cast<uint16_t>(scan_range.getEnd().value())));
  EXPECT_TRUE(DecodingEquals(data, static_cast<size_t>(Offset::MASTER_ANGLE_RESOLUTION), static_cast<uint16_t>(2)));

  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_ONE_START_ANGLE), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_ONE_END_ANGLE), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_ONE_ANGLE_RESOLUTION), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_TWO_START_ANGLE), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_TWO_END_ANGLE), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_TWO_ANGLE_RESOLUTION), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_THREE_START_ANGLE), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_THREE_END_ANGLE), 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, static_cast<size_t>(Offset::SLAVE_THREE_ANGLE_RESOLUTION), 0));
}

TEST_F(StartRequestTest, regressionForRealSystem)
{
  ScannerConfiguration sc(
      "192.168.0.50", 55115, 0, "192.168.0.10", DefaultScanRange(TenthOfDegree(0), TenthOfDegree(2750)), false);
  StartRequest sr(sc, 0);

  auto data = sr.serialize();

  unsigned char expected_crc[4] = { 0xaf, 0xc8, 0xde, 0x79 };  // see wireshark for this number

  for (size_t i = 0; i < sizeof(expected_crc); i++)
  {
    EXPECT_EQ(static_cast<unsigned int>(static_cast<unsigned char>(data[i])), expected_crc[i]);
  }
}

TEST_F(StartRequestTest, regressionForRealSystemWithDiagnostic)
{
  ScannerConfiguration sc(
      "192.168.0.50", 55115, 0, "192.168.0.10", DefaultScanRange(TenthOfDegree(0), TenthOfDegree(2750)), true);
  StartRequest sr(sc, 0);

  auto data = sr.serialize();

  unsigned char expected_crc[4] = { 0x18, 0x5b, 0xd5, 0x55 };  // see wireshark for this number

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
