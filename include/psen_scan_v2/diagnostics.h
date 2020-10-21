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

#include "psen_scan_v2/scanner_ids.h"

namespace psen_scan_v2
{
static constexpr uint32_t RAW_DIAGNOSTIC_MESSAGE_LENGTH_FOR_ONE_DEVICE_IN_BYTES{ 9 };
static constexpr uint32_t RAW_DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES{ 4 };
static constexpr uint32_t RAW_DIAGNOSTIC_MESSAGE_LENGTH_IN_BYTES{
  RAW_DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES +
  RAW_DIAGNOSTIC_MESSAGE_LENGTH_FOR_ONE_DEVICE_IN_BYTES * VALID_SCANNER_IDS.size()
};

using RawDiagnosticMsg = std::array<uint8_t, RAW_DIAGNOSTIC_MESSAGE_LENGTH_IN_BYTES>;

enum class DiagnosticCode
{
  ossd1_oc,
  ossd_shrt_c,
  ossd_integr,
  intern,
  win_cln_al,
  power_supply,
  netw_prb,
  dust_crc_fl,
  ossd2_overcur,
  meas_prob,
  incoherence,
  zone_inval_trans,
  zone_invalid_conf,
  win_cln_warn,
  int_com_prb,
  generic_err,
  disp_com_prb,
  temp_meas_prob,
  encod_oor,
  edm2_err,
  edm1_err,
  conf_err,
  out_of_range_err,
  temp_range_err,
  encoder_generic_err,
  unused
};

// clang-format off

using Dc = DiagnosticCode;
using ErrorMessage = std::string;

static const std::map<DiagnosticCode, ErrorMessage> error_code_to_string
{
  { Dc::ossd1_oc, "OSSD1 Overcurrent / Short circuit." },
  { Dc::ossd_shrt_c, "Short circuit between at least two OSSDs." },
  { Dc::ossd_integr, "Integrity check problem on any OSSD" },
  { Dc::intern, "Internal error." },
  { Dc::win_cln_al, "Alarm: The front panel of the safety laser scanner must be cleaned." },
  { Dc::power_supply, "Power supply problem." },
  { Dc::netw_prb, "Network problem." },
  { Dc::dust_crc_fl, "Dust circuit failure" },
  { Dc::ossd2_overcur, "OSSD2 Overcurrent / Short circuit." },
  { Dc::meas_prob, "Measurement Problem." },
  { Dc::incoherence, "Incoherence Error" },
  { Dc::zone_inval_trans, "Zone: Invalid input - transition or integrity." },
  { Dc::zone_invalid_conf, "Zone: Invalid input configuration / connection." },
  { Dc::win_cln_warn, "Warning: The front panel of the safety laser scanner must be cleaned." },
  { Dc::int_com_prb, "Internal communication problem." },
  { Dc::generic_err, "Generic Error." },
  { Dc::disp_com_prb, "Display communication problem." },
  { Dc::temp_meas_prob, "Temperature measurement problem." },
  { Dc::encod_oor, "Encoder: Out of range." },
  { Dc::edm2_err, "Error in the External Device Monitoring (edm2_err)." },
  { Dc::edm1_err, "Error in the External Device Monitoring (edm1_err)." },
  { Dc::conf_err, "Configuration Error." },
  { Dc::out_of_range_err, "Out of range error." },
  { Dc::temp_range_err, "Temperature out of range." },
  { Dc::encoder_generic_err, "Encoder: Generic error." },
  { Dc::unused, "Unexpected error" } \
};

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<DiagnosticCode, 8>, 9> error_bits{{
  //Bit7                 Bit6              Bit5              Bit4              Bit3              Bit2                  Bit1                   Bit0
  { REV(Dc::ossd1_oc,    Dc::ossd_shrt_c,  Dc::ossd_integr,  Dc::intern,       Dc::intern,       Dc::intern,           Dc::intern,            Dc::intern) },
  { REV(Dc::win_cln_al,  Dc::power_supply, Dc::netw_prb,     Dc::dust_crc_fl,  Dc::intern,       Dc::intern,           Dc::unused,            Dc::ossd2_overcur) },
  { REV(Dc::meas_prob,   Dc::intern,       Dc::intern,       Dc::intern,       Dc::incoherence,  Dc::zone_inval_trans, Dc::zone_invalid_conf, Dc::win_cln_warn) },
  { REV(Dc::int_com_prb, Dc::intern,       Dc::intern,       Dc::generic_err,  Dc::disp_com_prb, Dc::intern,           Dc::intern,            Dc::temp_meas_prob) },
  { REV(Dc::encod_oor,   Dc::unused,       Dc::unused,       Dc::edm2_err,     Dc::edm1_err,     Dc::conf_err,         Dc::out_of_range_err,  Dc::temp_range_err) },
  { REV(Dc::unused,      Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,           Dc::unused,            Dc::encoder_generic_err) },
  { REV(Dc::unused,      Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,           Dc::unused,            Dc::unused) },
  { REV(Dc::unused,      Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,           Dc::unused,            Dc::unused) },
  { REV(Dc::unused,      Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,       Dc::unused,           Dc::unused,            Dc::unused) },
  }};
// clang-format on

class ErrorLocation
{
public:
  using ByteLocation = size_t;
  using BitLocation = size_t;
  constexpr ErrorLocation(const ByteLocation& byte, const BitLocation& bit) : byte_(byte), bit_(bit){};
  inline constexpr ByteLocation getByte() const
  {
    return byte_;
  };
  inline constexpr BitLocation getBit() const
  {
    return bit_;
  };

private:
  ByteLocation byte_;
  BitLocation bit_;
};

class MonitoringFrameDiagnosticMessage
{
public:
  constexpr MonitoringFrameDiagnosticMessage(const ScannerId& id, const ErrorLocation& location);

  constexpr bool operator==(const MonitoringFrameDiagnosticMessage& rhs) const;

  friend RawDiagnosticMsg serializeDiagnosticMessages(const std::vector<MonitoringFrameDiagnosticMessage>& messages);

  constexpr ScannerId getScannerId() const
  {
    return id_;
  }

  constexpr ErrorLocation getErrorLocation() const
  {
    return error_location_;
  }

  constexpr DiagnosticCode getDiagnosticCode() const
  {
    return error_bits.at(error_location_.getByte()).at(error_location_.getBit());
  }

private:
  ScannerId id_;
  ErrorLocation error_location_;
};

constexpr inline MonitoringFrameDiagnosticMessage::MonitoringFrameDiagnosticMessage(const ScannerId& id,
                                                                                    const ErrorLocation& location)
  : id_(id), error_location_(location)
{
}

constexpr inline bool MonitoringFrameDiagnosticMessage::operator==(const MonitoringFrameDiagnosticMessage& rhs) const
{
  return (error_location_.getBit() == rhs.error_location_.getBit() &&
          error_location_.getByte() == rhs.error_location_.getByte() && id_ == rhs.id_);
}

// Store ambiguous errors for additional output
static const std::set<Dc> ambiguous_diagnostic_codes = { Dc::unused, Dc::intern };

inline bool isAmbiguous(const DiagnosticCode& code)
{
  return ambiguous_diagnostic_codes.find(code) != ambiguous_diagnostic_codes.end();
}

std::ostream& operator<<(std::ostream& os, const MonitoringFrameDiagnosticMessage& msg);

}  //  namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_DIAGNOSTICS_H
