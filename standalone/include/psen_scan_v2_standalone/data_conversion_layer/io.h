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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_H
#define PSEN_SCAN_V2_STANDALONE_IO_H

#include <array>

namespace psen_scan_v2_standalone
{
/**
 * @brief Contains the data serialization and deserialization layer.
 */
namespace data_conversion_layer
{
namespace monitoring_frame
{
/**
 * @brief Contains all types, etc. needed to describe the IOs information contained
 * in a  data_conversion_layer::monitoring_frame::Message.
 */
namespace io
{
/**
 * @brief Contains constants and types needed to define the io::Message.
 */
static constexpr uint32_t RAW_CHUNK_LENGTH_RESERVED_IN_BYTES{ 4 };
static constexpr uint32_t RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES{ 10 };
static constexpr uint32_t RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES{ 8 };
static constexpr uint32_t RAW_CHUNK_LENGTH_IN_BYTES{ RAW_CHUNK_LENGTH_RESERVED_IN_BYTES +
                                                     RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES +
                                                     RAW_CHUNK_LENGTH_RESERVED_IN_BYTES +
                                                     RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES };

using RawChunk = std::array<uint8_t, io::RAW_CHUNK_LENGTH_IN_BYTES>;

enum class PhysicalInputType
{
  zone_sw_8,
  zone_sw_7,
  zone_sw_6,
  zone_sw_5,
  zone_sw_4,
  zone_sw_3,
  zone_sw_2,
  zone_sw_1,
  override_12,
  override_11,
  muting_12,
  muting_11,
  muting_en_1,
  restart_1,
  reset,
  edm_2,
  override_22,
  override_21,
  muting_22,
  muting_21,
  muting_en_2,
  restart_2,
  edm_1,
  edm_3,
  override_32,
  override_31,
  muting_32,
  muting_31,
  muting_en_3,
  restart_3,
  unused
};

using Pit = PhysicalInputType;

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<Pit, 8>, RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES> error_bits{{
  //    Bit7              Bit6              Bit5              Bit4              Bit3              Bit2              Bit1              Bit0
  { REV(Pit::zone_sw_8,   Pit::zone_sw_7,   Pit::zone_sw_6,   Pit::zone_sw_5,   Pit::zone_sw_4,   Pit::zone_sw_3,   Pit::zone_sw_2,   Pit::zone_sw_1) },
  { REV(Pit::override_12, Pit::override_11, Pit::muting_12,   Pit::muting_11,   Pit::muting_en_1, Pit::restart_1,   Pit::unused,      Pit::reset) },
  { REV(Pit::edm_2,       Pit::override_22, Pit::override_21, Pit::muting_22,   Pit::muting_21,   Pit::muting_en_2, Pit::restart_2,   Pit::edm_1) },
  { REV(Pit::unused,      Pit::edm_3,       Pit::override_32, Pit::override_31, Pit::muting_32,   Pit::muting_31,   Pit::muting_en_3, Pit::restart_3) },
  }};
// clang-format on

// TODO: LogicalInputType ...

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_H