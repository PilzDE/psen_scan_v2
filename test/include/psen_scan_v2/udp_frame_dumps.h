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
#include <vector>
#include <map>
#include <iostream>

#include "psen_scan_v2/monitoring_frame_msg.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
namespace scanner_udp_datagram_hexdumps
{
constexpr uint8_t
clearIntensityChannelBits(const size_t index, const size_t begin, const size_t n, const uint8_t hexdump_byte)
{
  if ((index >= begin) && (index < begin + n) && (0 == index % 2))
  {
    return 0b00111111 & hexdump_byte;
  }
  else
  {
    return hexdump_byte;
  }
}

constexpr uint16_t convertHexdumpBytesToUint16_t(const uint8_t msbyte, const uint8_t lsbyte)
{
  return static_cast<uint16_t>(msbyte << 8) + static_cast<uint16_t>(lsbyte);
}

template <size_t ARRAY_SIZE>
inline std::vector<double> readMeasurements(const std::array<uint8_t, ARRAY_SIZE> hex_dump,
                                            const size_t offset_measurements,
                                            const size_t n_measurements)
{
  std::vector<double> measurements;
  for (size_t idx = offset_measurements; idx < (offset_measurements + (n_measurements * 2)); idx = idx + 2)
  {
    uint16_t raw_value = convertHexdumpBytesToUint16_t(hex_dump.at(idx + 1), hex_dump.at(idx));
    measurements.push_back(raw_value / 1000.);
  }
  return measurements;
}

template <size_t ARRAY_SIZE>
inline std::vector<double> readIntensities(const std::array<uint8_t, ARRAY_SIZE> hex_dump,
                                           const size_t offset_measurements,
                                           const size_t n_measurements)
{
  std::vector<double> measurements;
  for (size_t idx = offset_measurements; idx < (offset_measurements + (n_measurements * 2)); idx = idx + 2)
  {
    uint16_t raw_value = convertHexdumpBytesToUint16_t(hex_dump.at(idx + 1), hex_dump.at(idx));
    measurements.push_back(raw_value & 0b0011111111111111);
  }
  return measurements;
}

class WithIntensitiesAndDiagnostics
{
public:
  WithIntensitiesAndDiagnostics()
  {
    monitoring_frame::Message msg(
        TenthOfDegree(0x3e8),
        TenthOfDegree(0x02),
        0x00008894,
        readMeasurements(hex_dump, 74, 250),
        readIntensities(hex_dump, intensities_offset, 250),
        { monitoring_frame::diagnostic::Message(ScannerId::master, monitoring_frame::diagnostic::ErrorLocation(2, 0)),
          monitoring_frame::diagnostic::Message(ScannerId::master,
                                                monitoring_frame::diagnostic::ErrorLocation(4, 3)) });
    expected_msg_ = msg;
  };

  const size_t intensities_offset{ 74 + 3 + (250 * 2) };
  // clang-format off
  const std::array<uint8_t, 1081> hex_dump = {
                                                               0x00, 0x00, 0x00, 0x00, 0xca, 0x00, // 0020
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xe8, 0x03, 0x02, 0x00, 0x02, // 0030
   0x05, 0x00, 0x94, 0x88, 0x00, 0x00, 0x04, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // 0040
   0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0050
   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0060
   0x00, 0x05, 0xf5, 0x01, 0xf8, 0x02, 0xfd, 0x02, 0xff, 0x02, 0x01, 0x03, 0x03, 0x03, 0x02, 0x03, // 0070
   0x03, 0x03, 0x07, 0x03, 0x0a, 0x03, 0x0e, 0x03, 0x0c, 0x03, 0x0b, 0x03, 0x0a, 0x03, 0x15, 0x03, // 0080
   0x1d, 0x03, 0x12, 0x03, 0x12, 0x03, 0x19, 0x03, 0x22, 0x03, 0x1e, 0x03, 0x23, 0x03, 0x25, 0x03, // 0090
   0x22, 0x03, 0x26, 0x03, 0x28, 0x03, 0x30, 0x03, 0x0e, 0x03, 0xbc, 0x02, 0x85, 0x02, 0x90, 0x02, // 00a0
   0x9a, 0x02, 0x90, 0x02, 0x7b, 0x02, 0x84, 0x02, 0x7a, 0x02, 0x70, 0x02, 0x70, 0x02, 0x7b, 0x02, // 00b0
   0x70, 0x02, 0x70, 0x02, 0x64, 0x02, 0x70, 0x02, 0x7b, 0x02, 0x64, 0x02, 0x7b, 0x02, 0x70, 0x02, // 00c0
   0x7b, 0x02, 0x7b, 0x02, 0x7b, 0x02, 0x92, 0x02, 0x7c, 0x02, 0x7c, 0x02, 0x87, 0x02, 0x87, 0x02, // 00d0
   0x92, 0x02, 0x7c, 0x02, 0x7c, 0x02, 0x9d, 0x02, 0x92, 0x02, 0x88, 0x02, 0x88, 0x02, 0x88, 0x02, // 00e0
   0x92, 0x02, 0x93, 0x02, 0x93, 0x02, 0x9e, 0x02, 0x9e, 0x02, 0x7f, 0x02, 0x7c, 0x02, 0x7c, 0x02, // 00f0
   0x7d, 0x02, 0x7d, 0x02, 0x88, 0x02, 0x88, 0x02, 0x7d, 0x02, 0x88, 0x02, 0x88, 0x02, 0x7d, 0x02, // 0100
   0x88, 0x02, 0x72, 0x02, 0x88, 0x02, 0x88, 0x02, 0x94, 0x02, 0x89, 0x02, 0x89, 0x02, 0x94, 0x02, // 0110
   0x94, 0x02, 0x94, 0x02, 0x9f, 0x02, 0x9f, 0x02, 0x9f, 0x02, 0x89, 0x02, 0x95, 0x02, 0xa0, 0x02, // 0120
   0xa0, 0x02, 0x95, 0x02, 0x95, 0x02, 0xab, 0x02, 0x95, 0x02, 0x95, 0x02, 0xa0, 0x02, 0x96, 0x02, // 0130
   0xa0, 0x02, 0xa1, 0x02, 0xa1, 0x02, 0xa1, 0x02, 0xa1, 0x02, 0x96, 0x02, 0xa1, 0x02, 0xac, 0x02, // 0140
   0xa1, 0x02, 0x97, 0x02, 0x97, 0x02, 0x97, 0x02, 0x97, 0x02, 0x97, 0x02, 0x93, 0x02, 0x93, 0x02, // 0150
   0x97, 0x02, 0x93, 0x02, 0x9f, 0x02, 0xaa, 0x02, 0xaa, 0x02, 0xc5, 0x02, 0xc5, 0x02, 0xae, 0x02, // 0160
   0xc5, 0x02, 0xd0, 0x02, 0xc5, 0x02, 0xd0, 0x02, 0xd0, 0x02, 0xd0, 0x02, 0xd0, 0x02, 0xd0, 0x02, // 0170
   0xdb, 0x02, 0xdb, 0x02, 0xd1, 0x02, 0xdc, 0x02, 0xe7, 0x02, 0xdc, 0x02, 0xdc, 0x02, 0xc6, 0x02, // 0180
   0xe7, 0x02, 0xe7, 0x02, 0xdc, 0x02, 0xe7, 0x02, 0xc6, 0x02, 0xd1, 0x02, 0xc7, 0x02, 0xd2, 0x02, // 0190
   0xd2, 0x02, 0xc7, 0x02, 0xdd, 0x02, 0xdd, 0x02, 0xdd, 0x02, 0xdd, 0x02, 0xdd, 0x02, 0xfe, 0x02, // 01a0
   0xf3, 0x02, 0xf3, 0x02, 0xfe, 0x02, 0x09, 0x03, 0xfd, 0x02, 0x9f, 0x03, 0x1e, 0x04, 0x98, 0x04, // 01b0
   0x98, 0x04, 0x86, 0x04, 0xec, 0x04, 0xe2, 0x04, 0xe7, 0x04, 0xeb, 0x04, 0xed, 0x04, 0xf6, 0x04, // 01c0
   0x54, 0x05, 0xc6, 0x06, 0xdb, 0x06, 0xf6, 0x06, 0xff, 0x06, 0x0f, 0x07, 0x1f, 0x07, 0x36, 0x07, // 01d0
   0x43, 0x07, 0x52, 0x07, 0x66, 0x07, 0x78, 0x07, 0x93, 0x07, 0xa4, 0x07, 0xab, 0x07, 0xc1, 0x07, // 01e0
   0xde, 0x07, 0xec, 0x07, 0x00, 0x08, 0x17, 0x08, 0x30, 0x08, 0x3f, 0x08, 0x5d, 0x08, 0x73, 0x08, // 01f0
   0x8f, 0x08, 0x9f, 0x08, 0xc8, 0x08, 0xde, 0x08, 0xf0, 0x08, 0x0f, 0x09, 0x23, 0x09, 0x3f, 0x09, // 0200
   0x5a, 0x09, 0x76, 0x09, 0x9c, 0x09, 0xbe, 0x09, 0xe2, 0x09, 0x04, 0x0a, 0x2d, 0x0a, 0x4c, 0x0a, // 0210
   0x72, 0x0a, 0x9d, 0x0a, 0xc7, 0x0a, 0xed, 0x0a, 0x1b, 0x0b, 0x44, 0x0b, 0x70, 0x0b, 0xa4, 0x0b, // 0220
   0xd9, 0x0b, 0x05, 0x0c, 0x51, 0x0c, 0x83, 0x0c, 0xbf, 0x0c, 0xbd, 0x0c, 0xba, 0x0c, 0xba, 0x0c, // 0230
   0xba, 0x0c, 0xb1, 0x0c, 0xbf, 0x0c, 0xa6, 0x0c, 0xa6, 0x0c, 0x9c, 0x0c, 0xa0, 0x0c, 0x9b, 0x0c, // 0240
   0x9d, 0x0c, 0x97, 0x0c, 0x9a, 0x0c, 0x99, 0x0c, 0xa2, 0x0c, 0x99, 0x0c, 0x94, 0x0c, 0x90, 0x0c, // 0250
   0x93, 0x0c, 0x93, 0x0c, 0x92, 0x0c, 0x91, 0x0c, 0x06, 0xf5, 0x01, 0x80, 0x8a, 0x80, 0x8a, 0x70, // 0260
   0x8a, 0x7c, 0x8a, 0x7b, 0x8a, 0x70, 0x8a, 0x66, 0x8a, 0x7c, 0x8a, 0x86, 0x8a, 0x6a, 0x8a, 0x5e, // 0270
   0x8a, 0x67, 0x8a, 0x77, 0x8a, 0x76, 0x8a, 0x72, 0x8a, 0x7e, 0x8a, 0x62, 0x8a, 0x5d, 0x8a, 0x66, // 0280
   0x8a, 0x60, 0x8a, 0x67, 0x8a, 0x60, 0x8a, 0x64, 0x8a, 0x74, 0x8a, 0x2f, 0x8a, 0x64, 0x89, 0xe4, // 0290
   0x49, 0x3f, 0x49, 0xed, 0x45, 0x18, 0x45, 0xf3, 0x44, 0xd8, 0x44, 0x76, 0x45, 0x8e, 0x47, 0x99, // 02a0
   0x48, 0xd6, 0x48, 0xe3, 0x48, 0xe2, 0x48, 0xe8, 0x48, 0xe7, 0x48, 0xe9, 0x48, 0xd7, 0x48, 0xd3, // 02b0
   0x48, 0xe4, 0x48, 0xca, 0x48, 0xdb, 0x48, 0xcb, 0x48, 0xce, 0x48, 0xc2, 0x48, 0xbc, 0x48, 0xc4, // 02c0
   0x48, 0xb4, 0x48, 0xbb, 0x48, 0xb5, 0x48, 0x9c, 0x48, 0xa8, 0x48, 0xad, 0x48, 0x93, 0x48, 0x99, // 02d0
   0x48, 0x97, 0x48, 0x91, 0x48, 0x87, 0x48, 0x85, 0x48, 0x86, 0x48, 0x72, 0x48, 0x6a, 0x48, 0x69, // 02e0
   0x48, 0x68, 0x48, 0x68, 0x48, 0x50, 0x48, 0x52, 0x48, 0x5b, 0x48, 0x4f, 0x48, 0x44, 0x48, 0x3c, // 02f0
   0x48, 0x40, 0x48, 0x33, 0x48, 0x2e, 0x48, 0x2c, 0x48, 0x24, 0x48, 0x1e, 0x48, 0x12, 0x48, 0x1e, // 0300
   0x48, 0x13, 0x48, 0x19, 0x48, 0x0e, 0x48, 0x09, 0x48, 0x08, 0x48, 0xfc, 0x47, 0xfd, 0x47, 0xf1, // 0310
   0x47, 0x00, 0x48, 0x06, 0x48, 0xf0, 0x47, 0xfd, 0x47, 0xfd, 0x47, 0xfd, 0x47, 0xf8, 0x47, 0xfa, // 0320
   0x47, 0x02, 0x48, 0xf9, 0x47, 0x06, 0x48, 0x0d, 0x48, 0x0c, 0x48, 0x12, 0x48, 0x0e, 0x48, 0x23, // 0330
   0x48, 0x22, 0x48, 0x1f, 0x48, 0x1b, 0x48, 0x27, 0x48, 0x3a, 0x48, 0x4b, 0x48, 0x41, 0x48, 0x5a, // 0340
   0x48, 0x51, 0x48, 0x6c, 0x48, 0x73, 0x48, 0x63, 0x48, 0x7a, 0x48, 0x77, 0x48, 0x77, 0x48, 0x71, // 0350
   0x48, 0x7b, 0x48, 0x8b, 0x48, 0x74, 0x48, 0x88, 0x48, 0x89, 0x48, 0x96, 0x48, 0x7b, 0x48, 0x8f, // 0360
   0x48, 0x8d, 0x48, 0x8d, 0x48, 0x89, 0x48, 0x95, 0x48, 0x97, 0x48, 0x99, 0x48, 0x8e, 0x48, 0x7b, // 0370
   0x48, 0x82, 0x48, 0x8c, 0x48, 0x76, 0x48, 0x84, 0x48, 0x82, 0x48, 0x90, 0x48, 0x79, 0x48, 0x71, // 0380
   0x48, 0x72, 0x48, 0x62, 0x48, 0x61, 0x48, 0x56, 0x48, 0x72, 0x48, 0x62, 0x48, 0x3d, 0x48, 0x47, // 0390
   0x48, 0x45, 0x48, 0x41, 0x48, 0x2c, 0x48, 0x1f, 0x48, 0x24, 0x48, 0xfd, 0x47, 0xb7, 0x47, 0x3c, // 03a0
   0x46, 0x0e, 0x43, 0xf9, 0x41, 0x2c, 0x41, 0x9a, 0x43, 0xc1, 0x48, 0xc2, 0x87, 0x76, 0x88, 0x78, // 03b0
   0x88, 0x53, 0x88, 0x41, 0x88, 0x3f, 0x88, 0xc1, 0x87, 0xbd, 0x87, 0x14, 0x88, 0xfd, 0x87, 0x00, // 03c0
   0x88, 0xd2, 0x87, 0xcd, 0x87, 0xcb, 0x87, 0xae, 0x87, 0xa2, 0x87, 0x69, 0x87, 0x89, 0x87, 0x5a, // 03d0
   0x87, 0x48, 0x87, 0x4c, 0x87, 0x05, 0x87, 0x24, 0x87, 0x1d, 0x87, 0xb8, 0x86, 0xd9, 0x86, 0xa3, // 03e0
   0x86, 0x9f, 0x86, 0x98, 0x86, 0x74, 0x86, 0x66, 0x86, 0x3b, 0x86, 0x60, 0x86, 0x32, 0x86, 0xfd, // 03f0
   0x85, 0xf2, 0x85, 0xa3, 0x85, 0xa9, 0x85, 0x9e, 0x85, 0x8f, 0x85, 0xae, 0x85, 0x8b, 0x85, 0x6f, // 0400
   0x85, 0x47, 0x85, 0x1c, 0x85, 0xfd, 0x84, 0xb8, 0x84, 0xac, 0x84, 0xa1, 0x84, 0x8c, 0x84, 0x49, // 0410
   0x84, 0x48, 0x84, 0x63, 0x84, 0x1a, 0x84, 0x10, 0x84, 0x18, 0x84, 0x11, 0x14, 0x2d, 0x13, 0x76, // 0420
   0x16, 0xe7, 0x87, 0xbd, 0x87, 0x9c, 0x87, 0x97, 0x87, 0x58, 0x87, 0x5f, 0x87, 0xfb, 0x86, 0xf5, // 0430
   0x86, 0x92, 0x86, 0xbf, 0x86, 0xca, 0x86, 0xe5, 0x86, 0xda, 0x86, 0xe8, 0x86, 0xe7, 0x86, 0xfd, // 0440
   0x86, 0x05, 0x87, 0xff, 0x86, 0x11, 0x87, 0x1e, 0x87, 0x28, 0x87, 0x30, 0x87, 0x35, 0x87, 0x09, // 0450
                                                                                 0x00, 0x00, 0x00  // 0460
  };
  // clang-format on

  monitoring_frame::Message expected_msg_;
};

class WithoutMeasurementsAndIntensities
{
public:
  WithoutMeasurementsAndIntensities()
  {
    monitoring_frame::Message msg(TenthOfDegree(0x5dc), TenthOfDegree(0x0a), 0x0661fc, {});
    expected_msg_ = msg;
  }

  const std::array<uint8_t, 35> hex_dump = { 0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00, // End of FixedFields
                                             0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,
                                             0x05, 0x01, 0x00,
                                             0x09, 0x00, 0x00, 0x00 };

  monitoring_frame::Message expected_msg_;
};

class WithUnknownFieldId
{
public:
  const std::array<uint8_t, 35> hex_dump = { 0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00, // End of FixedFields
                                             0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,
                                             0x0a, 0x01, 0x00,
                                             0x09, 0x00, 0x00, 0x00 };
};

class WithTooLargeFieldLength
{
public:
  const std::array<uint8_t, 35> hex_dump = { 0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00, // End of FixedFields
                                             0x02, 0x05, 0x00, 0xfc, 0x61, 0x06, 0x00,
                                             0x05, 0xcf, 0xff,
                                             0x09, 0x00, 0x00, 0x00 };
};

class WithTooLargeScanCounterLength
{
private:
  unsigned char modv{ 0x06 };

public:
  const std::array<uint8_t, 35> hex_dump = { 0x00, 0x00, 0x00, 0x00, 0xca, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                             0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x05, 0x0a, 0x00, // End of FixedFields
                                             0x02, modv, 0x00, 0xfc, 0x61, 0x06, 0x00,
                                             0x05, 0x00, 0x00,
                                             0x09, 0x00, 0x00, 0x00 };
};
}  // namespace scanner_udp_datagram_hexdumps
}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_UDP_FRAME_DUMPS_H
