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

#include "psen_scan_v2/diagnostics.h"

using namespace psen_scan_v2;

namespace psen_scan_v2
{
MonitoringFrameDiagnosticMessage::MonitoringFrameDiagnosticMessage(ScannerId id,
                                                                   DiagnoseFieldErrorByteLocation byte_location,
                                                                   DiagnoseFieldErrorBitLocation bit_location)
  : id_(id)
  , code_(error_bits[byte_location][bit_location])
  , byte_location_(byte_location)
  , bit_location_(bit_location)
  , level_(MonitoringFrameDiagnosticMessageLevel::ERROR)
{
}

std::ostream& operator<<(std::ostream& os, const MonitoringFrameDiagnosticMessage& msg)
{
  os << fmt::format("{} {} {} {}", msg.id_, msg.code_, msg.byte_location_, msg.bit_location_);
  return os;
}

typedef DiagnosticCode Dc;

std::map<DiagnosticCode, ErrorCodeName> error_code_to_string{ { Dc::OSSD1_OC, "OSSD1_OC" },
                                                              { Dc::OSSD_SHRT_C, "OSSD_SHORT_CIRC" },
                                                              { Dc::OSSD_INTEGR, "OSSD_INTEGRITY" },
                                                              { Dc::INT, "INTERNAL" },
                                                              { Dc::WIN_CLN_AL, "WIN_CLN_AL" },
                                                              { Dc::POWER_SUPPLY, "POWER_SUPPLY" },
                                                              { Dc::NETW_PRB, "NETWORK_PROBLEM" },
                                                              { Dc::DUST_CRC_FL, "DUST_CIRC_FAIL" },
                                                              { Dc::OSSD2_OVERCUR, "OSSD2_OVERCUR" },
                                                              { Dc::MEAS_PROB, "MEAS_PROB" },
                                                              { Dc::INCOHERENCE, "INCOHERENCE" },
                                                              { Dc::ZONE_INVAL_TRANS, "ZONE_INVALID_TRANS" },
                                                              { Dc::ZONE_INVALID_CONF, "ZONE_INVALID_CONF" },
                                                              { Dc::WIN_CLN_WARN, "WINDOW_CLEAN_WARN" },
                                                              { Dc::INT_COM_PRB, "INTERN_COMMUNCATION_PROB" },
                                                              { Dc::GENERIC_ERR, "GENERIC_ERR" },
                                                              { Dc::DISP_COM_PRB, "DISPLAY_COM_PROB" },
                                                              { Dc::TEMP_MEAS_PROB, "TEMP_MEAS_PROB" },
                                                              { Dc::ENCOD_OOR, "ENCOD_OOR" },
                                                              { Dc::EDM2_ERR, "EDM2_ERR" },
                                                              { Dc::EDM1_ERR, "EDM1_ERR" },
                                                              { Dc::CONF_ERR, "CONF_ERR" },
                                                              { Dc::OUT_OF_RANGE_ERR, "OUT_OF_RANGE_ERR" },
                                                              { Dc::TEMP_RANGE_ERR, "TEMP_RANGE_ERR" },
                                                              { Dc::ENCODER_GENERIC_ERR, "ENCODER_GENERIC_ERR" },
                                                              { Dc::_, "UNEXPECTED" } };

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  std::array<std::array<DiagnosticCode, 8>, 9> error_bits{{
  //Bit8                 Bit7              Bit6              Bit5              Bit4              Bit3                  Bit2                   Bit0
  { REV(Dc::OSSD1_OC,    Dc::OSSD_SHRT_C,  Dc::OSSD_INTEGR,  Dc::INT,          Dc::INT,          Dc::INT,              Dc::INT,               Dc::INT) },
  { REV(Dc::WIN_CLN_AL,  Dc::POWER_SUPPLY, Dc::NETW_PRB,     Dc::DUST_CRC_FL,  Dc::INT,          Dc::INT,              Dc::_,                 Dc::OSSD2_OVERCUR) },
  { REV(Dc::MEAS_PROB,   Dc::INT,          Dc::INT,          Dc::INT,          Dc::INCOHERENCE,  Dc::ZONE_INVAL_TRANS, Dc::ZONE_INVALID_CONF, Dc::WIN_CLN_WARN) },
  { REV(Dc::INT_COM_PRB, Dc::INT,          Dc::INT,          Dc::GENERIC_ERR,  Dc::DISP_COM_PRB, Dc::INT,              Dc::INT,               Dc::TEMP_MEAS_PROB) },
  { REV(Dc::ENCOD_OOR,   Dc::_,            Dc::_,            Dc::EDM2_ERR,     Dc::EDM1_ERR,     Dc::CONF_ERR,         Dc::OUT_OF_RANGE_ERR,  Dc::TEMP_RANGE_ERR) },
  { REV(Dc::_,           Dc::_,            Dc::_,            Dc::_,            Dc::_,            Dc::_,                Dc::_,                 Dc::ENCODER_GENERIC_ERR) },
  { REV(Dc::UNUSED,      Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,           Dc::UNUSED,            Dc::UNUSED) },
  { REV(Dc::UNUSED,      Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,           Dc::UNUSED,            Dc::UNUSED) },
  { REV(Dc::UNUSED,      Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,       Dc::UNUSED,           Dc::UNUSED,            Dc::UNUSED) },
  }};
// clang-format on
}  // namespace psen_scan_v2