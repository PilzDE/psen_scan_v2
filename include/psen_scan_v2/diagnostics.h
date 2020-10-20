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

#ifndef PSEN_SCAN_V2_DIAGNOSTICS_H
#define PSEN_SCAN_V2_DIAGNOSTICS_H

#include <array>
#include <string>
#include <map>
#include <vector>
#include <set>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2/scanner_ids.h"

namespace psen_scan_v2
{
static constexpr uint32_t DIAGNOSTIC_MESSAGE_RAW_LENGTH_FOR_ONE_DEVICE_IN_BYTES{ 9 };
static constexpr uint32_t DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES{ 4 };
static constexpr uint32_t DIAGNOSTIC_DATA_LENGTH_IN_BYTES{ DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES +
                                                           DIAGNOSTIC_MESSAGE_RAW_LENGTH_FOR_ONE_DEVICE_IN_BYTES *
                                                               VALID_SCANNER_IDS.size() };

using RawDiagnosticMsg = std::array<uint8_t, DIAGNOSTIC_DATA_LENGTH_IN_BYTES>;

enum class DiagnosticCode
{
  OSSD1_OC,
  OSSD_SHRT_C,
  OSSD_INTEGR,
  INT,
  WIN_CLN_AL,
  POWER_SUPPLY,
  NETW_PRB,
  DUST_CRC_FL,
  OSSD2_OVERCUR,
  MEAS_PROB,
  INCOHERENCE,
  ZONE_INVAL_TRANS,
  ZONE_INVALID_CONF,
  WIN_CLN_WARN,
  INT_COM_PRB,
  GENERIC_ERR,
  DISP_COM_PRB,
  TEMP_MEAS_PROB,
  ENCOD_OOR,
  EDM2_ERR,
  EDM1_ERR,
  CONF_ERR,
  OUT_OF_RANGE_ERR,
  TEMP_RANGE_ERR,
  ENCODER_GENERIC_ERR,
  _,
  UNUSED
};

// clang-format off

using Dc = DiagnosticCode;
using ErrorMessage = std::string;

static const std::map<DiagnosticCode, ErrorMessage> error_code_to_string
{
  { Dc::OSSD1_OC, "OSSD1 Overcurrent / Short circuit." },
  { Dc::OSSD_SHRT_C, "Short circuit between at least two OSSDs." },
  { Dc::OSSD_INTEGR, "Integrity check problem on any OSSD" },
  { Dc::INT, "Internal error." },
  { Dc::WIN_CLN_AL, "Alarm: The front panel of the safety laser scanner must be cleaned." },
  { Dc::POWER_SUPPLY, "Power supply problem." },
  { Dc::NETW_PRB, "Network problem." },
  { Dc::DUST_CRC_FL, "Dust circuit failure" },
  { Dc::OSSD2_OVERCUR, "OSSD2 Overcurrent / Short circuit." },
  { Dc::MEAS_PROB, "Measurement Problem." },
  { Dc::INCOHERENCE, "Incoherence Error" },
  { Dc::ZONE_INVAL_TRANS, "Zone: Invalid input - transition or integrity." },
  { Dc::ZONE_INVALID_CONF, "Zone: Invalid input configuration / connection." },
  { Dc::WIN_CLN_WARN, "Warning: The front panel of the safety laser scanner must be cleaned." },
  { Dc::INT_COM_PRB, "Internal communication problem." },
  { Dc::GENERIC_ERR, "Generic Error." },
  { Dc::DISP_COM_PRB, "Display communication problem." },
  { Dc::TEMP_MEAS_PROB, "Temperature measurement problem." },
  { Dc::ENCOD_OOR, "Encoder: Out of range." },
  { Dc::EDM2_ERR, "Error in the External Device Monitoring (EDM2_ERR)." },
  { Dc::EDM1_ERR, "Error in the External Device Monitoring (EDM1_ERR)." },
  { Dc::CONF_ERR, "Configuration Error." },
  { Dc::OUT_OF_RANGE_ERR, "Out of range error." },
  { Dc::TEMP_RANGE_ERR, "Temperature out of range." },
  { Dc::ENCODER_GENERIC_ERR, "Encoder: Generic error." },
  { Dc::_, "Unexpected error" } \
};

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<DiagnosticCode, 8>, 9> error_bits{{
  //Bit7                 Bit6              Bit5              Bit4              Bit3              Bit2                  Bit1                   Bit0
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

class ErrorLocation
{
public:
  using byteLocation = size_t;
  using bitLocation = size_t;
  ErrorLocation(byteLocation byte, bitLocation bit) : byte_(byte), bit_(bit){};
  inline byteLocation getByte() const
  {
    return byte_;
  };
  inline bitLocation getBit() const
  {
    return bit_;
  };

private:
  byteLocation byte_;
  bitLocation bit_;
};

class MonitoringFrameDiagnosticMessage
{
public:
  MonitoringFrameDiagnosticMessage(ScannerId id, ErrorLocation location);

  bool operator==(const MonitoringFrameDiagnosticMessage& rhs) const;

  friend RawDiagnosticMsg serializeDiagnosticMessages(std::vector<MonitoringFrameDiagnosticMessage>& messages);

  ScannerId getScannerId() const
  {
    return id_;
  }

  ErrorLocation getErrorLocation() const
  {
    return error_location_;
  }

  DiagnosticCode getDiagnosticCode() const
  {
    return error_bits.at(error_location_.getByte()).at(error_location_.getBit());
  }

private:
  ScannerId id_;
  ErrorLocation error_location_;
};

// Store ambiguous errors for additional output
static const std::set<Dc> ambiguous_diagnostic_codes = { Dc::_, Dc::UNUSED, Dc::INT };

inline bool isAmbiguous(const DiagnosticCode& code)
{
  return ambiguous_diagnostic_codes.find(code) != ambiguous_diagnostic_codes.end();
}

std::ostream& operator<<(std::ostream& os, const MonitoringFrameDiagnosticMessage& msg);

}  //  namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_DIAGNOSTICS_H
