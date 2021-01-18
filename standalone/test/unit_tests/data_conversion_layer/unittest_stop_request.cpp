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

#include <boost/crc.hpp>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/stop_request_serialization.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_test_helper.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
class StopRequestTest : public ::testing::Test
{
};

TEST_F(StopRequestTest, constructorTest)
{
  const auto raw_stop_request{ data_conversion_layer::stop_request::serialize() };
  boost::crc_32_type crc;
  crc.process_bytes(&raw_stop_request[sizeof(uint32_t)], raw_stop_request.size() - sizeof(uint32_t));

  EXPECT_TRUE(DecodingEquals(raw_stop_request, 0x00, static_cast<uint32_t>(crc.checksum())))
      << "Calculated CRC incorrect";

  const std::size_t crc_offset{ 4u };
  for (std::size_t i = 0; i < data_conversion_layer::stop_request::NUM_RESERVED_FIELDS; ++i)
  {
    const std::size_t curr_field_offset{ crc_offset + i * sizeof(uint8_t) };
    EXPECT_TRUE(DecodingEquals(raw_stop_request, curr_field_offset, static_cast<uint8_t>(0)))
        << "Reserved field has incorrect value";
  }

  const std::size_t op_code_offset{ crc_offset +
                                    data_conversion_layer::stop_request::NUM_RESERVED_FIELDS * sizeof(uint8_t) };
  EXPECT_TRUE(DecodingEquals(raw_stop_request, op_code_offset, static_cast<uint32_t>(0x36))) << "OP code incorrect";
}

TEST_F(StopRequestTest, crcShouldBeCorrectAfterSerialization)
{
  const auto raw_stop_request{ data_conversion_layer::stop_request::serialize() };
  const std::array<unsigned char, 4> expected_crc = { 0x28, 0xec, 0xfb, 0x39 };  // see wireshark for this number
  for (size_t i = 0; i < expected_crc.size(); ++i)
  {
    EXPECT_EQ(static_cast<unsigned int>(static_cast<unsigned char>(raw_stop_request[i])), expected_crc[i]);
  }
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
