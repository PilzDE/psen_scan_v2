// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_TEST_UDP_DATA_READER_H
#define PSEN_SCAN_V2_TEST_UDP_DATA_READER_H

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace psen_scan_v2_test
{
struct UdpDatum
{
  uint32_t scan_counter_;
  uint16_t from_theta_;
  double timestamp_sec_;
};

using UdpData = std::vector<UdpDatum>;

class UdpDataReader
{
public:
  static void read(const std::string& filename, const uint16_t port, UdpData& udp_data);

private:
  static bool readAndCheckPortNumber(std::istringstream& is, const uint16_t expected_port);
  template <typename T>
  static void readHexValue(std::istringstream& is, const std::string& expected_key, T& value);
  static void readTimestamp(std::istringstream& is, double& value);
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_UDP_DATA_READER_H
