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

#ifndef PSEN_SCAN_V2_DIAGNOSTIC_H
#define PSEN_SCAN_V2_DIAGNOSTIC_H

#include <array>
#include <string>
#include <map>

namespace psen_scan_v2
{

// clang-format off
enum class DiagnosticCode
{
  OSSD1_OVERCUR,
  OSSD_SHORT_CIRC,
  OSSD_INTEGRITY,
  INTERNAL,
  WINDOW_CLEAN_ALARM,
  POWER_SUPPLY,
  NETWORK_PROBLEM,
  DUST_CIRC_FAIL,
  OSSD2_OVERCUR,
  MEAS_PROB,
  INCOHERENCE,
  ZONE_INVALID_TRANS,
  ZONE_INVALID_CONF,
  WINDOW_CLEAN_WARN,
  INT_COM_PROB,
  GENERIC_ERR,
  DISPLAY_COM_PROB,
  TEMP_MEAS_PROB,
  ENCOD_OUT_RANGE,
  EDM2_ERR,
  EDM1_ERR,
  CONF_ERR,
  OUT_OF_RANGE_ERR,
  TEMP_RANGE_ERR,
  ENCODER_GENERIC_ERR,
  _,
  UNUSED
};

  typedef DiagnosticCode Dc;

  std::map<DiagnosticCode, std::string> error_code_to_string { { Dc::OSSD1_OVERCUR, "OSSD1_OVERCUR" },
                                                               { Dc::OSSD_SHORT_CIRC, "OSSD_SHORT_CIRC" },
                                                               { Dc::OSSD_INTEGRITY, "OSSD_INTEGRITY"},
                                                               { Dc::INTERNAL, "INTERNAL"},
                                                               { Dc::WINDOW_CLEAN_ALARM, "WINDOW_CLEAN_ALARM"},
                                                               { Dc::POWER_SUPPLY, "POWER_SUPPLY"},
                                                               { Dc::NETWORK_PROBLEM, "NETWORK_PROBLEM"},
                                                               { Dc::DUST_CIRC_FAIL, "DUST_CIRC_FAIL"},
                                                               { Dc::OSSD2_OVERCUR, "OSSD2_OVERCUR"},
                                                               { Dc::MEAS_PROB, "MEAS_PROB"},
                                                               { Dc::INCOHERENCE, "INCOHERENCE"},
                                                               { Dc::ZONE_INVALID_TRANS, "ZONE_INVALID_TRANS"},
                                                               { Dc::ZONE_INVALID_CONF, "ZONE_INVALID_CONF"},
                                                               { Dc::WINDOW_CLEAN_WARN, "WINDOW_CLEAN_WARN"},
                                                               { Dc::INT_COM_PROB, "INTERN_COMMUNCATION_PROB"},
                                                               { Dc::GENERIC_ERR, "GENERIC_ERR"},
                                                               { Dc::DISPLAY_COM_PROB, "DISPLAY_COM_PROB"},
                                                               { Dc::TEMP_MEAS_PROB, "TEMP_MEAS_PROB"},
                                                               { Dc::ENCOD_OUT_RANGE, "ENCOD_OUT_RANGE"},
                                                               { Dc::EDM2_ERR, "EDM2_ERR"},
                                                               { Dc::EDM1_ERR, "EDM1_ERR"},
                                                               { Dc::CONF_ERR, "CONF_ERR"},
                                                               { Dc::OUT_OF_RANGE_ERR, "OUT_OF_RANGE_ERR"},
                                                               { Dc::TEMP_RANGE_ERR, "TEMP_RANGE_ERR"},
                                                               { Dc::ENCODER_GENERIC_ERR, "ENCODER_GENERIC_ERR"},
                                                               { Dc::_, "UNEXPECTED"}
                                                              };

  std::array<std::array<DiagnosticCode, 8>, 9> error_bits{{
  //Bit0                      Bit1                   Bit2                    Bit3                  Bit4                 Bit5                 Bit6                   Bit7
  { Dc::INTERNAL,             Dc::INTERNAL,          Dc::INTERNAL,           Dc::INTERNAL,         Dc::INTERNAL,        Dc::OSSD_INTEGRITY,  Dc::OSSD_SHORT_CIRC,   Dc::OSSD1_OVERCUR },
  { Dc::OSSD2_OVERCUR,        Dc::_,                 Dc::INTERNAL,           Dc::INTERNAL,         Dc::DUST_CIRC_FAIL,  Dc::NETWORK_PROBLEM, Dc::POWER_SUPPLY,      Dc::WINDOW_CLEAN_ALARM },
  { Dc::WINDOW_CLEAN_WARN,    Dc::ZONE_INVALID_CONF, Dc::ZONE_INVALID_TRANS, Dc::INCOHERENCE,      Dc::INTERNAL,        Dc::INTERNAL,        Dc::INTERNAL,          Dc::MEAS_PROB },
  { Dc::TEMP_MEAS_PROB,       Dc::INTERNAL,          Dc::INTERNAL,           Dc::DISPLAY_COM_PROB, Dc::GENERIC_ERR,     Dc::INTERNAL,        Dc::INTERNAL,          Dc::INT_COM_PROB },
  { Dc::TEMP_RANGE_ERR,       Dc::OUT_OF_RANGE_ERR,  Dc::CONF_ERR,           Dc::EDM1_ERR,         Dc::EDM2_ERR,        Dc::_,               Dc::_,                 Dc::ENCOD_OUT_RANGE },
  { Dc::ENCODER_GENERIC_ERR,  Dc::_,                 Dc::_,                  Dc::_,                Dc::_,               Dc::_,               Dc::_,                 Dc::_ },
  { Dc::UNUSED,               Dc::UNUSED,            Dc::UNUSED,             Dc::UNUSED,           Dc::UNUSED,          Dc::UNUSED,          Dc::UNUSED,            Dc::UNUSED },
  { Dc::UNUSED,               Dc::UNUSED,            Dc::UNUSED,             Dc::UNUSED,           Dc::UNUSED,          Dc::UNUSED,          Dc::UNUSED,            Dc::UNUSED },
  { Dc::UNUSED,               Dc::UNUSED,            Dc::UNUSED,             Dc::UNUSED,           Dc::UNUSED,          Dc::UNUSED,          Dc::UNUSED,            Dc::UNUSED },
  }};

// clang-format on

}  //  namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LOGGING_H