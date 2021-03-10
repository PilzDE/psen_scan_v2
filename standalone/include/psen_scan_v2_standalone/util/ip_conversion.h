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

#ifndef PSEN_SCAN_V2_STANDALONE_UTIL_IP_CONVERSION_H
#define PSEN_SCAN_V2_STANDALONE_UTIL_IP_CONVERSION_H

#include <string>
#include <cstdint>
#include <cassert>
#include <stdexcept>
#include <limits>

#ifdef __linux__
#include <arpa/inet.h>
#endif
#ifdef _WIN32
#include <WinSock2.h>
#endif

namespace psen_scan_v2_standalone
{
namespace util
{
#ifdef __linux__
inline uint32_t convertIP(const std::string& ip)
{
  const auto ip_number = inet_network(ip.c_str());
  if (static_cast<in_addr_t>(-1) == ip_number)
  {
    throw std::invalid_argument("IP invalid");
  }
  assert(sizeof(ip_number) == 4 && "ip_number has not the expected size");
  return static_cast<uint32_t>(ip_number);
}
#endif

#ifdef _WIN32
inline uint32_t convertIP(const std::string& ip)
{
  const auto ip_number = inet_addr(ip.c_str());
  if (INADDR_NONE == ip_number)
  {
    throw std::invalid_argument("IP invalid");
  }
  assert(sizeof(ip_number) == 4 && "ip_number has not the expected size");
  return htonl(static_cast<uint32_t>(ip_number));
}
#endif

inline uint16_t convertPort(const int& port)
{
  if (port < std::numeric_limits<uint16_t>::min() || port > std::numeric_limits<uint16_t>::max())
  {
    throw std::out_of_range("Port out of range");
  }
  return static_cast<uint16_t>(port);
}

}  // namespace util
}  // namespace psen_scan_v2_standalone

#endif
