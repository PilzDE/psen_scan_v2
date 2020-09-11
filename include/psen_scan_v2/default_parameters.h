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

#include "psen_scan_v2/degree_to_rad.h"

namespace psen_scan_v2
{
static const std::string DEFAULT_FRAME_ID = "scanner";

//! @brief Start angle of measurement.
static constexpr double DEFAULT_ANGLE_START(0.0);
//! @brief  End angle of measurement.
static constexpr double DEFAULT_ANGLE_END(degreeToRad(275.));

//! @brief Rotation of x-axis around the center.
static constexpr double DEFAULT_X_AXIS_ROTATION(degreeToRad(137.5));

//! @brief Topic on which the LaserScan data are published.
static const std::string DEFAULT_PUBLISH_TOPIC = "scan";

}  // namespace psen_scan_v2
#endif  // PSEN_SCAN_V2_DEFAULT_PARAMETERS_H
