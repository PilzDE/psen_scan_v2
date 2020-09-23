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

#ifndef PSEN_SCAN_V2_DEFAULT_PARAMETERS_H
#define PSEN_SCAN_V2_DEFAULT_PARAMETERS_H

#include <string>

#include "psen_scan_v2/angle_conversions.h"

namespace psen_scan_v2
{
//! @brief Start angle of measurement.
static constexpr double DEFAULT_ANGLE_START(-degreeToRadian(137.5));
//! @brief  End angle of measurement.
static constexpr double DEFAULT_ANGLE_END(degreeToRadian(137.5));

}  // namespace psen_scan_v2
#endif  // PSEN_SCAN_V2_DEFAULT_PARAMETERS_H
