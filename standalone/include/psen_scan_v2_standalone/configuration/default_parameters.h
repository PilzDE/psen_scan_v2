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

#ifndef PSEN_SCAN_V2_STANDALONE_DEFAULT_PARAMETERS_H
#define PSEN_SCAN_V2_STANDALONE_DEFAULT_PARAMETERS_H

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"

namespace psen_scan_v2_standalone
{
namespace configuration
{
static const std::string DEFAULT_HOST_IP_STRING{ "auto" };

static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };

static constexpr unsigned short DATA_PORT_OF_HOST_DEVICE{ 55115 };
static constexpr unsigned short CONTROL_PORT_OF_HOST_DEVICE{ 55116 };

static constexpr bool FRAGMENTED_SCANS{ false };
static constexpr bool INTENSITIES{ false };
static constexpr bool DIAGNOSTICS{ false };

//! @brief Start angle of measurement.
static constexpr double DEFAULT_ANGLE_START(-data_conversion_layer::degreeToRadian(137.4));
//! @brief  End angle of measurement.
static constexpr double DEFAULT_ANGLE_END(data_conversion_layer::degreeToRadian(137.4));
static constexpr double DEFAULT_SCAN_ANGLE_RESOLUTION(data_conversion_layer::degreeToRadian(0.1));

static constexpr double TIME_PER_SCAN_IN_S{ 0.03 };

static constexpr double RANGE_MIN_IN_M{ 0.05 };
static constexpr double RANGE_MAX_IN_M{ 40. };

static constexpr double DEFAULT_X_AXIS_ROTATION(data_conversion_layer::degreeToRadian(137.5));
}  // namespace configuration

}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_DEFAULT_PARAMETERS_H
