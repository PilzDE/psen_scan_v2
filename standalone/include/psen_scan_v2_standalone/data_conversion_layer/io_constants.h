// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_CONSTANTS_H
#define PSEN_SCAN_V2_STANDALONE_IO_CONSTANTS_H

#include <array>
#include <map>
#include <string>

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
namespace io
{
/**
 * @brief Contains constants and types needed to define PinData and for the (de)serialization.
 */
static constexpr uint32_t RAW_CHUNK_LENGTH_RESERVED_IN_BYTES{ 4 };
static constexpr uint32_t RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES{ 10 };
static constexpr uint32_t RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES{ 8 };
static constexpr uint32_t RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES{ 4 };
static constexpr uint32_t RAW_CHUNK_LENGTH_IN_BYTES{
  3 * (RAW_CHUNK_LENGTH_RESERVED_IN_BYTES + RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES) +
  RAW_CHUNK_LENGTH_RESERVED_IN_BYTES + RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES + RAW_CHUNK_LENGTH_RESERVED_IN_BYTES +
  RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES
};

using RawChunk = std::array<uint8_t, io::RAW_CHUNK_LENGTH_IN_BYTES>;

#define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

enum class LogicalInputType
{
  zone_bit_0,
  zone_bit_1,
  zone_bit_2,
  zone_bit_3,
  zone_bit_4,
  zone_bit_5,
  zone_bit_6,
  zone_bit_7,

  zone_sw_2,
  zone_sw_1,
  overr_2_a,
  overr_1_a,
  muting_2_a,
  muting_1_a,

  reset_a,
  zone_sw_8,
  zone_sw_7,
  zone_sw_6,
  zone_sw_5,
  zone_sw_4,
  zone_sw_3,

  cor_seq_mut_2,
  mut_en_2_a,
  restart_2_a,
  cor_seq_or_1,
  cor_seq_mut_1,
  mut_en_1_a,
  restart_1_a,

  cor_seq_or_2,
  unused
};

using Lit = LogicalInputType;
using IoName = std::string;
// clang-format off
static const std::map<Lit, IoName> LOGICAL_INPUT_BIT_TO_NAME{
  { Lit::zone_bit_0, "Zone Bit 0" },
  { Lit::zone_bit_1, "Zone Bit 1" },
  { Lit::zone_bit_2, "Zone Bit 2" },
  { Lit::zone_bit_3, "Zone Bit 3" },
  { Lit::zone_bit_4, "Zone Bit 4" },
  { Lit::zone_bit_5, "Zone Bit 5" },
  { Lit::zone_bit_6, "Zone Bit 6" },
  { Lit::zone_bit_7, "Zone Bit 7" },

  { Lit::zone_sw_2, "Zone Set Switching Input 2" },
  { Lit::zone_sw_1, "Zone Set Switching Input 1" },
  { Lit::overr_2_a, "Override 2 Activated" },
  { Lit::overr_1_a, "Override 1 Activated" },
  { Lit::muting_2_a, "Muting 2 Activated" },
  { Lit::muting_1_a, "Muting 1 Activated" },

  { Lit::reset_a, "Reset Activated" },
  { Lit::zone_sw_8, "Zone Set Switching Input 8" },
  { Lit::zone_sw_7, "Zone Set Switching Input 7" },
  { Lit::zone_sw_6, "Zone Set Switching Input 6" },
  { Lit::zone_sw_5, "Zone Set Switching Input 5" },
  { Lit::zone_sw_4, "Zone Set Switching Input 4" },
  { Lit::zone_sw_3, "Zone Set Switching Input 3" },

  { Lit::cor_seq_mut_2, "Correct activation sequence of Muting 2 Pins" },
  { Lit::mut_en_2_a, "Muting Enable 2 Activated" },
  { Lit::restart_2_a, "Restart 2 Activated" },
  { Lit::cor_seq_or_1, "Correct activation sequence of Override 1 Pins" },
  { Lit::cor_seq_mut_1, "Correct activation sequence of Muting 1 Pins" },
  { Lit::mut_en_1_a, "Muting Enable 1 Activated" },
  { Lit::restart_1_a, "Restart 1 Activated" },

  { Lit::cor_seq_or_2, "Correct activation sequence of Override 2 Pins" },
  { Lit::unused, "unused" },
};

static constexpr std::array<std::array<Lit, 8>, RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES> LOGICAL_INPUT_BITS{{
  //    Bit7                Bit6             Bit5              Bit4             Bit3               Bit2                Bit1             Bit0
  { REV(Lit::zone_bit_7,    Lit::zone_bit_6, Lit::zone_bit_5,  Lit::zone_bit_4, Lit::zone_bit_3,   Lit::zone_bit_2,    Lit::zone_bit_1, Lit::zone_bit_0) },
  {     Lit::unused,        Lit::unused,     Lit::unused,      Lit::unused,     Lit::unused,       Lit::unused,        Lit::unused,     Lit::unused },
  {     Lit::unused,        Lit::unused,     Lit::unused,      Lit::unused,     Lit::unused,       Lit::unused,        Lit::unused,     Lit::unused },
  {     Lit::unused,        Lit::unused,     Lit::unused,      Lit::unused,     Lit::unused,       Lit::unused,        Lit::unused,     Lit::unused },
  { REV(Lit::zone_sw_2,     Lit::zone_sw_1,  Lit::unused,      Lit::overr_2_a,  Lit::overr_1_a,    Lit::unused,        Lit::muting_2_a, Lit::muting_1_a) },
  { REV(Lit::unused,        Lit::reset_a,    Lit::zone_sw_8,   Lit::zone_sw_7,  Lit::zone_sw_6,    Lit::zone_sw_5,     Lit::zone_sw_4,  Lit::zone_sw_3) },
  { REV(Lit::cor_seq_mut_2, Lit::mut_en_2_a, Lit::restart_2_a, Lit::unused,     Lit::cor_seq_or_1, Lit::cor_seq_mut_1, Lit::mut_en_1_a, Lit::restart_1_a) },
  { REV(Lit::unused,        Lit::unused,     Lit::unused,      Lit::unused,     Lit::unused,       Lit::unused,        Lit::unused,     Lit::cor_seq_or_2) },
}};
// clang-format on

enum class OutputType
{
  unused,
  ossd1_refpts,

  warn_2_int,
  warn_1_int,
  ossd3_lock,
  safe_3_int,
  int_lock_2,
  safe_2_int,
  int_lock_1,
  safe_1_int
};

using Ot = OutputType;

// clang-format off
static const std::map<Ot, IoName> OUTPUT_BIT_TO_NAME
{
  { Ot::unused, "unused" },
  { Ot::ossd1_refpts, "REFERENCE POINTS VIOLATION" },

  { Ot::warn_2_int, "Warning 2 intrusion" },
  { Ot::warn_1_int, "Warning 1 intrusion" },
  { Ot::safe_3_int, "Safety 3 intrusion" },
  { Ot::int_lock_2, "INTERLOCK 2" },
  { Ot::safe_2_int, "Safety 2 intrusion" },
  { Ot::int_lock_1, "INTERLOCK 1" },
  { Ot::safe_1_int, "Safety 1 intrusion" }
};

static constexpr std::array<std::array<Ot, 8>, RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES> OUTPUT_BITS{{
  //      Bit7            Bit6            Bit5          Bit4              Bit3            Bit2            Bit1            Bit0
  { REV(Ot::warn_2_int, Ot::warn_1_int, Ot::unused,   Ot::safe_3_int,   Ot::int_lock_2, Ot::safe_2_int, Ot::int_lock_1, Ot::safe_1_int) },
  { REV(Ot::unused,     Ot::unused,     Ot::unused,   Ot::unused,       Ot::unused,     Ot::unused,     Ot::unused,     Ot::unused) },
  { REV(Ot::unused,     Ot::unused,     Ot::unused,   Ot::unused,       Ot::unused,     Ot::unused,     Ot::unused,     Ot::unused) },
  { REV(Ot::unused,     Ot::unused,     Ot::unused,   Ot::ossd1_refpts, Ot::unused,     Ot::unused,     Ot::unused,     Ot::unused) }
}};
// clang-format on

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_CONSTANTS_H
