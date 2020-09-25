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

#include <boost/crc.hpp>

#include <gtest/gtest.h>

#include "psen_scan_v2/stop_request.h"
#include "psen_scan_v2/raw_data_test_helper.h"
#include "psen_scan_v2/scanner_constants.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
class StopRequestTest : public ::testing::Test
{
};

TEST_F(StopRequestTest, constructorTest)
{
  StopRequest request;
  auto data{ request.toRawData() };
  boost::crc_32_type crc;
  crc.process_bytes(&data[sizeof(uint32_t)], data.size() - sizeof(uint32_t));

  EXPECT_TRUE(DecodingEquals(data, OFFSET_CRC, static_cast<uint32_t>(crc.checksum()))) << "Calculated CRC incorrect";

  const std::size_t reserved_offset{ OFFSET_RESERVED };
  for (std::size_t i = 0; i < stop_request_constants::NUM_RESERVED_FIELDS; ++i)
  {
    const std::size_t curr_field_offset{ reserved_offset + i * sizeof(uint8_t) };
    EXPECT_TRUE(DecodingEquals(data, curr_field_offset, static_cast<uint8_t>(0)))
        << "Reserved field has incorrect value";
  }

  const std::size_t op_code_offset{ reserved_offset + stop_request_constants::NUM_RESERVED_FIELDS * sizeof(uint8_t) };
  EXPECT_TRUE(DecodingEquals(data, op_code_offset, static_cast<uint32_t>(0x36))) << "OP code incorrect";
}

TEST_F(StopRequestTest, regressionForRealSystem)
{
  StopRequest sr;

  auto data = sr.toRawData();

  unsigned char expected_crc[4] = { 0x28, 0xec, 0xfb, 0x39 };  // see wireshark for this number

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
