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
#include "psen_scan_v2/scanner_constants.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2::start_request_constants;

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

  EXPECT_TRUE(DecodingEquals(data, OFFSET_CRC, (uint32_t)result.checksum()));

  EXPECT_TRUE(DecodingEquals(data, OFFSET_CRC, 0xd6224cb9));  // CRC - Fixed for now, Note:
                                                              // Other byte order as in
                                                              // wireshark

  EXPECT_TRUE(DecodingEquals(data, OFFSET_SEQ_NUMBER, (uint32_t)sequence_number));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_RESERVED, (uint64_t)0));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_OPCODE, (uint32_t)0x35));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_IP, inet_network(host_ip.c_str()), Endian::BIG));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_UDP_PORT, host_udp_port_data));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_DEVICE_ENABLED, (uint8_t)0b00001000));

  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_INTENSITIES_ENABLED, 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_POINT_IN_SAFETY_ENABLED, 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_ACTIVE_ZONE_SET_ENABLED, 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_IO_PIN_ENABLED, 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_SCAN_COUNTER_ENABLED, 0b00001000));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_SPEED_ENCODER_ENABLED, 0));
  EXPECT_TRUE(DecodingEquals<uint8_t>(data, OFFSET_DIAGNOSTICS_ENABLED, 0));

  EXPECT_TRUE(DecodingEquals(data, OFFSET_MASTER_START_ANGLE, radToTenthDegree(start_angle)));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_MASTER_END_ANGLE, radToTenthDegree(end_angle)));
  EXPECT_TRUE(DecodingEquals(data, OFFSET_MASTER_ANGLE_RESOLUTION, degreeToTenthDegree(0.1)));

  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_ONE_START_ANGLE, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_ONE_END_ANGLE, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_ONE_ANGLE_RESOLUTION, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_TWO_START_ANGLE, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_TWO_END_ANGLE, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_TWO_ANGLE_RESOLUTION, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_THREE_START_ANGLE, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_THREE_END_ANGLE, 0));
  EXPECT_TRUE(DecodingEquals<uint16_t>(data, OFFSET_SLAVE_THREE_ANGLE_RESOLUTION, 0));
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
