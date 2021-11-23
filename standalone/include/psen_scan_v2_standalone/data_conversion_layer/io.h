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
#include <map>
#include <vector>

#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2_standalone
{
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
static constexpr uint32_t RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES{ 4 };
static constexpr uint32_t RAW_CHUNK_LENGTH_IN_BYTES{
  RAW_CHUNK_LENGTH_RESERVED_IN_BYTES + RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES + RAW_CHUNK_LENGTH_RESERVED_IN_BYTES +
  RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES + RAW_CHUNK_LENGTH_RESERVED_IN_BYTES + RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES
};

using RawChunk = std::array<uint8_t, io::RAW_CHUNK_LENGTH_IN_BYTES>;

// clang-format off
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
using IoName = std::string;

static const std::map<Pit, IoName> physical_input_bit_to_name
{
  { Pit::zone_sw_8, "Zone Set Switching Input 8" },
  { Pit::zone_sw_7, "Zone Set Switching Input 7" },
  { Pit::zone_sw_6, "Zone Set Switching Input 6" },
  { Pit::zone_sw_5, "Zone Set Switching Input 5" },
  { Pit::zone_sw_4, "Zone Set Switching Input 4" },
  { Pit::zone_sw_3, "Zone Set Switching Input 3" },
  { Pit::zone_sw_2, "Zone Set Switching Input 2" },
  { Pit::zone_sw_1, "Zone Set Switching Input 1" },

  { Pit::override_12, "Override 12" },
  { Pit::override_11, "Override 11" },
  { Pit::muting_12, "Muting 12" },
  { Pit::muting_11, "Muting 11" },
  { Pit::muting_en_1, "Muting Enable 1" },
  { Pit::restart_1, "Restart 1" },
  { Pit::unused, "unused" },
  { Pit::reset, "Reset" },

  { Pit::edm_2, "EDM 2" },
  { Pit::override_22, "Override 22" },
  { Pit::override_21, "Override 21" },
  { Pit::muting_22, "Muting 22" },
  { Pit::muting_21, "Muting 21" },
  { Pit::muting_en_2, "Muting Enable 2" },
  { Pit::restart_2, "Restart 2" },
  { Pit::edm_1, "EDM 1" },

  { Pit::edm_3, "EDM 3" },
  { Pit::override_32, "Override 32" },
  { Pit::override_31, "Override 31" },
  { Pit::muting_32, "Muting 32" },
  { Pit::muting_31, "Muting 31" },
  { Pit::muting_en_3, "Muting Enable 3" },
  { Pit::restart_3, "Restart 3" },
};

  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<Pit, 8>, RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES> physical_input_bits{{
  //    Bit7              Bit6              Bit5              Bit4              Bit3              Bit2              Bit1              Bit0
  { REV(Pit::zone_sw_8,   Pit::zone_sw_7,   Pit::zone_sw_6,   Pit::zone_sw_5,   Pit::zone_sw_4,   Pit::zone_sw_3,   Pit::zone_sw_2,   Pit::zone_sw_1) },
  { REV(Pit::override_12, Pit::override_11, Pit::muting_12,   Pit::muting_11,   Pit::muting_en_1, Pit::restart_1,   Pit::unused,      Pit::reset) },
  { REV(Pit::edm_2,       Pit::override_22, Pit::override_21, Pit::muting_22,   Pit::muting_21,   Pit::muting_en_2, Pit::restart_2,   Pit::edm_1) },
  { REV(Pit::unused,      Pit::edm_3,       Pit::override_32, Pit::override_31, Pit::muting_32,   Pit::muting_31,   Pit::muting_en_3, Pit::restart_3) },
  }};

// TODO: LogicalInputType ...

enum class OutputType
{
  unused,
  ossd1_refpts,
  warn2_slv3,
  warn1_slv3,
  ossd3_slv3,
  ossd2_slv3,

  ossd1_slv3,
  warn2_slv2,
  warn1_slv2,
  ossd3_slv2,
  ossd2_slv2,
  ossd1_slv2,
  warn2_slv1,
  warn1_slv1,

  ossd3_slv1,
  ossd2_slv1,
  ossd1_slv1,
  warn2_m,
  warn1_m,
  ossd3_m,
  ossd2_m,
  ossd1_m,

  warn2,
  warn1,
  ossd3_lock,
  ossd3,
  ossd2_lock,
  ossd2,
  ossd1_lock,
  ossd1
};

using Ot = OutputType;

static const std::map<Ot, IoName> output_bit_to_name
{
  { Ot::unused, "unused" },
  { Ot::ossd1_refpts, "OSSD1_REFPTS" },
  { Ot::warn2_slv3, "WARN2_SLV3" },
  { Ot::warn1_slv3, "WARN1_SLV3" },
  { Ot::ossd3_slv3, "OSSD3_SLV3" },
  { Ot::ossd2_slv3, "OSSD2_SLV3" },
  
  { Ot::ossd1_slv3, "OSSD1_SLV3" },
  { Ot::warn2_slv2, "WARN2_SLV2" },
  { Ot::warn1_slv2, "WARN1_SLV2" },
  { Ot::ossd3_slv2, "OSSD3_SLV2" },
  { Ot::ossd2_slv2, "OSSD2_SLV2" },
  { Ot::ossd1_slv2, "OSSD1_SLV2" },
  { Ot::warn2_slv1, "WARN2_SLV1" },
  { Ot::warn1_slv1, "WARN1_SLV1" },
  
  { Ot::ossd3_slv1, "OSSD3_SLV1" },
  { Ot::ossd2_slv1, "OSSD2_SLV1" },
  { Ot::ossd1_slv1, "OSSD1_SLV1" },
  { Ot::warn2_m, "WARN2_M" },
  { Ot::warn1_m, "WARN1_M" },
  { Ot::ossd3_m, "OSSD3_M" },
  { Ot::ossd2_m, "OSSD2_M" },
  { Ot::ossd1_m, "OSSD1_M" },
  
  { Ot::warn2, "WARN2" },
  { Ot::warn1, "WARN1" },
  { Ot::ossd3_lock, "OSSD3_LOCK" },
  { Ot::ossd3, "OSSD3" },
  { Ot::ossd2_lock, "OSSD2_LOCK" },
  { Ot::ossd2, "OSSD2" },
  { Ot::ossd1_lock, "OSSD1_LOCK" },
  { Ot::ossd1, "OSSD1" } 
};

  static constexpr std::array<std::array<Ot, 8>, RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES> output_bits{{
  //    Bit7              Bit6              Bit5              Bit4              Bit3              Bit2              Bit1              Bit0
  { REV(Ot::unused,       Ot::unused,       Ot::unused,       Ot::ossd1_refpts, Ot::warn2_slv3,   Ot::warn1_slv3,   Ot::ossd3_slv3,   Ot::ossd2_slv3) },
  { REV(Ot::ossd1_slv3,   Ot::warn2_slv2,   Ot::warn1_slv2,   Ot::ossd3_slv2,   Ot::ossd2_slv2,   Ot::ossd1_slv2,   Ot::warn2_slv1,   Ot::warn1_slv1) },
  { REV(Ot::ossd3_slv1,   Ot::ossd2_slv1,   Ot::ossd1_slv1,   Ot::warn2_m,      Ot::warn1_m,      Ot::ossd3_m,      Ot::ossd2_m,      Ot::ossd1_m) },
  { REV(Ot::warn2,        Ot::warn1,        Ot::ossd3_lock,   Ot::ossd3,        Ot::ossd2_lock,   Ot::ossd2,        Ot::ossd1_lock,   Ot::ossd1) },
  }};  // TODO: Verify byte order
// clang-format on

static uint32_t createID(size_t byte_n, size_t bit_n)
{
  return byte_n * 8 + bit_n;
}

static PinState createInputPinState(size_t byte_n, size_t bit_n, bool value)
{
  auto id = createID(byte_n, bit_n);
  auto input_bit = physical_input_bits.at(byte_n).at(bit_n);
  auto name = physical_input_bit_to_name.at(input_bit);
  return PinState(id, name, value);
}

static PinState createLogicalPinState(size_t byte_n, size_t bit_n, bool value)
{
  auto id = createID(byte_n, bit_n);
  return PinState(id, "", value);
}

static PinState createOutputPinState(size_t byte_n, size_t bit_n, bool value)
{
  auto id = createID(byte_n, bit_n);
  auto input_bit = output_bits.at(byte_n).at(bit_n);
  auto name = output_bit_to_name.at(input_bit);
  return PinState(id, name, value);
}

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_H