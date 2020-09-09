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

#ifndef PSEN_SCAN_V2_SCANNER_PARAMETER_H
#define PSEN_SCAN_V2_SCANNER_PARAMETER_H

#include <endian.h>
#include <cstdint>

namespace psen_scan_v2
{
/**
 * @brief Opcode for StartMonitoring Frame
 *
 * 0x09 as 32bit Little Endian
 *
 */
uint32_t const START_MONITORING_OPCODE = htole32(9);

/**
 * @brief Fixed sequence for StartMonitoring Frame
 *
 * Byte sequence 1-0-0-0-1-0-0-0
 *
 */
uint64_t const START_MONITORING_FIXED_SEQUENCE = 0x0000000100000001;

/**
 * @brief Opcode for StopMonitoring Frame
 *
 * 0x12 as 32bit Little Endian
 *
 */
uint32_t const STOP_MONITORING_OPCODE = htole32(18);

uint32_t const MONITORING_FRAME_OPCODE = 0xC9;    /**< Constant 0xC9. Byte order: little endian */
uint16_t const MAX_NUMBER_OF_SAMPLES = 550;       /**< Maximum number of samples per UDP message from Laserscanner */
unsigned short const PSEN_SCAN_PORT_WRITE = 3000; /**< Port on which Laserscanner expects messages */
unsigned short const PSEN_SCAN_PORT_READ = 2000;  /**< Port from which Laserscanner sends messages */
uint16_t const NUMBER_OF_SAMPLES_FULL_SCAN_MASTER = 2750; /**< Number of samples for complete scan for master scanner*/
double const SCAN_TIME = 0.03;                            /**< Time per scan in seconds*/
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_PARAMETER_H
