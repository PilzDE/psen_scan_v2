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

#ifndef PSEN_SCAN_V2_TEST_UDP_DATA_HELPER_H
#define PSEN_SCAN_V2_TEST_UDP_DATA_HELPER_H

#include <cstdint>
#include <fstream>
#include <ios>
#include <sstream>
#include <string>
#include <vector>

#include <boost/endian/conversion.hpp>

#include <fmt/format.h>

#include "psen_scan_v2_standalone/util/logging.h"

namespace psen_scan_v2_test
{
namespace udp_data
{
struct UdpDatum
{
  uint32_t scan_counter_;
  uint16_t from_theta_;
  double timestamp_sec_;
};

using UdpData = std::vector<UdpDatum>;

template <typename T>
static void readHexValue(std::istringstream& is, const std::string& expected_key, T& value)
{
  std::string key;
  if (!(is >> key && key == expected_key && is >> std::hex >> value))
  {
    throw std::ios_base::failure(fmt::format("Could not read {} from udp data file.", expected_key));
  }
  boost::endian::big_to_native_inplace(value);
}

static void readTimestamp(std::istringstream& is, double& value)
{
  std::string key;
  if (!(is >> key && key == "time_epoch:" && is >> std::dec >> value))
  {
    throw std::ios_base::failure("Could not read timestamp from udp data file.");
  }
}

static bool readAndCheckPortNumber(std::istringstream& is, const uint16_t expected_port)
{
  std::string key;
  uint16_t udp_port;
  if (!(is >> key && key == "udp_port:" && is >> std::dec >> udp_port))
  {
    throw std::ios_base::failure("Could not read port number from udp data file.");
  }
  return udp_port == expected_port;
}

static void readLine(const std::string& line, const uint16_t port, UdpData& udp_data)
{
  try
  {
    UdpDatum udp_datum;

    std::istringstream is(line);
    if (readAndCheckPortNumber(is, port))
    {
      readTimestamp(is, udp_datum.timestamp_sec_);
      readHexValue(is, "scan_counter:", udp_datum.scan_counter_);
      readHexValue(is, "from_theta:", udp_datum.from_theta_);

      udp_data.push_back(udp_datum);
    }
  }
  catch (const std::ios_base::failure& f)
  {
    PSENSCAN_WARN("udp_data::readLine()", f.what());
  }
}

static void read(const std::string& filename, const uint16_t port, UdpData& udp_data)
{
  std::ifstream filestr{ filename };
  if (!filestr.is_open())
  {
    PSENSCAN_ERROR("udp_data::read()", "Could not open file {}", filename);
    return;
  }

  std::string line;
  while (std::getline(filestr, line))
  {
    readLine(line, port, udp_data);
  }
  PSENSCAN_INFO("udp_data::read()", "Read data from {} udp packets.", udp_data.size());
  filestr.close();
}

}  // namespace udp_data
}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_UDP_DATA_HELPER_H
