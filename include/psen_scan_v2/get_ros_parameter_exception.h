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

#ifndef PSEN_SCAN_V2_GET_ROS_PARAMETER_EXCEPTION_H
#define PSEN_SCAN_V2_GET_ROS_PARAMETER_EXCEPTION_H

#include <stdexcept>

namespace psen_scan_v2
{
/**
 * @brief Exception thrown if a parameter of a certain type was not set.
 */
class ParameterNotSet : public std::runtime_error
{
public:
  ParameterNotSet(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_GET_ROS_PARAMETER_EXCEPTION_H
