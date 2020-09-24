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

#ifndef PSEN_SCAN_V2_UDP_FRAME_DUMPS_H
#define PSEN_SCAN_V2_UDP_FRAME_DUMPS_H

#include <array>
#include <map>

namespace psen_scan_v2_test
{
struct UDPFrameTestDataWithoutIntensities
{
  const std::array<uint8_t, 135> hex_dump = {
    0x00, 0x00, 0x00, 0x00, 0xca, 0x00,                                                              // 0020
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x0a, 0x00, 0x02,  // 0030
    0x05, 0x00, 0x78, 0x06, 0x00, 0x00, 0x05, 0x65, 0x00, 0xa9, 0x02, 0x06, 0x03, 0x7c, 0x02, 0xfa,  // 0040
    0x01, 0xf0, 0x01, 0x0e, 0x02, 0x77, 0x01, 0xb4, 0x01, 0xd6, 0x01, 0xd8, 0x01, 0xe5, 0x01, 0xe1,  // 0050
    0x01, 0xd4, 0x01, 0xd1, 0x01, 0xc7, 0x01, 0xba, 0x01, 0xb9, 0x01, 0xaf, 0x01, 0xac, 0x01, 0xb5,  // 0060
    0x01, 0x90, 0x0c, 0xc2, 0x0c, 0xb8, 0x0c, 0xd6, 0x0c, 0xd5, 0x0d, 0xa7, 0x10, 0xb5, 0x0d, 0xc0,  // 0070
    0x0f, 0x45, 0x0e, 0xd2, 0x0f, 0xdf, 0x0f, 0xf7, 0x0e, 0x8d, 0x0e, 0xa9, 0x0e, 0x41, 0x09, 0x11,  // 0080
    0x09, 0x0e, 0x09, 0xe7, 0x08, 0xe7, 0x08, 0xdd, 0x08, 0xeb, 0x08, 0xeb, 0x08, 0xd4, 0x08, 0xf1,  // 0090
    0x08, 0x04, 0x09, 0xea, 0x0f, 0xea, 0x0f, 0x03, 0x10, 0xc6, 0x10, 0x31, 0x12, 0x09, 0x00, 0x00,  // 00a0
    0x00                                                                                             // 00b0
  };  // hex dump: successful_stop_request.pcapng md5 e800c4 No. 22 only data payload of the udp frame

  const uint16_t from_theta{ 1000 };
  const uint16_t resolution{ 10 };
  const uint32_t scan_counter{ 1656 };

  const uint16_t number_of_measures{ 50 };

  using MeasuresIndexValuePair = std::pair<std::size_t, uint16_t>;
  const std::array<MeasuresIndexValuePair, 8> measures{
    { { 0, 681 }, { 1, 774 }, { 2, 636 }, { 3, 506 }, { 4, 496 }, { 30, 4063 }, { 45, 4074 }, { 49, 4657 } }
  };
};

struct UDPFrameTestDataWithoutMeasurementsAndIntensities
{
  const std::array<uint8_t, 35> hex_dump = { 0x00, 0x00, 0x00, 0x00, 0xca, 0x00,  // 0020
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                                             0x00, 0xdc, 0x05, 0x0a, 0x00, 0x02, 0x05, 0x00, 0xfc, 0x61,
                                             0x06, 0x00, 0x05, 0x01, 0x00, 0x09, 0x00, 0x00, 0x00 };
};

struct UDPFrameTestDataWithUnknownFieldId
{
  const std::array<uint8_t, 35> hex_dump = { 0x00, 0x00, 0x00, 0x00, 0xca, 0x00,  // 0020
                                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                                             0x00, 0xdc, 0x05, 0x0a, 0x00, 0x02, 0x05, 0x00, 0xfc, 0x61,
                                             0x06, 0x00, 0x0A, 0x01, 0x00, 0x09, 0x00, 0x00, 0x00 };
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_UDP_FRAME_DUMPS_H
