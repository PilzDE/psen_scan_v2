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

#include "psen_scan_v2/angle_conversions.h"

namespace psen_scan_v2
{
namespace constants
{
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };

//! @brief Start angle of measurement.
static constexpr double DEFAULT_ANGLE_START(-degreeToRadian(137.5));
//! @brief  End angle of measurement.
static constexpr double DEFAULT_ANGLE_END(degreeToRadian(137.5));

static constexpr uint16_t NUMBER_OF_SAMPLES_FULL_SCAN_MASTER{ 2750 };

static constexpr double TIME_PER_SCAN_IN_S{ 0.03 };

static constexpr double RANGE_MIN_IN_M{ 0.05 };
static constexpr double RANGE_MAX_IN_M{ 40. };

static constexpr double DEFAULT_X_AXIS_ROTATION(degreeToRadian(137.5));

static std::string SCAN_FRAME_ID_SUFFIX{ "_scan" };
}  // namespace constants

}  // namespace psen_scan_v2
#endif  // PSEN_SCAN_V2_DEFAULT_PARAMETERS_H
