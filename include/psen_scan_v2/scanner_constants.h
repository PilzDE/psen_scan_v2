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

#ifndef PSEN_SCAN_V2_SCANNER_CONSTANTS_H
#define PSEN_SCAN_V2_SCANNER_CONSTANTS_H

#include "psen_scan_v2/angle_conversions.h"

namespace psen_scan_v2
{
static constexpr uint16_t NUMBER_OF_SAMPLES_FULL_SCAN_MASTER{ 2750 };

static constexpr double TIME_PER_SCAN_IN_S{ 0.03 };

static constexpr double RANGE_MIN_IN_M{ 0. };
static constexpr double RANGE_MAX_IN_M{ 10. };

static constexpr double DEFAULT_X_AXIS_ROTATION(degreeToRadian(137.5));
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONSTANTS_H
