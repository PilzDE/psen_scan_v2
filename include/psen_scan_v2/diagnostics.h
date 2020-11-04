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
namespace monitoring_frame
{
namespace diagnostic
{
namespace raw_message
{
static constexpr uint32_t LENGTH_FOR_ONE_DEVICE_IN_BYTES{ 9 };
static constexpr uint32_t UNUSED_OFFSET_IN_BYTES{ 4 };
static constexpr uint32_t LENGTH_IN_BYTES{ UNUSED_OFFSET_IN_BYTES +
                                           LENGTH_FOR_ONE_DEVICE_IN_BYTES * VALID_SCANNER_IDS.size() };
using Field = std::array<uint8_t, diagnostic::raw_message::LENGTH_IN_BYTES>;
}  // namespace raw_message

enum class ErrorType
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

using Et = ErrorType;
using ErrorMessage = std::string;

static const std::map<ErrorType, ErrorMessage> error_code_to_string
{
  { Et::ossd1_oc, "OSSD1 Overcurrent / Short circuit." },
  { Et::ossd_shrt_c, "Short circuit between at least two OSSDs." },
  { Et::ossd_integr, "Integrity check problem on any OSSD" },
  { Et::intern, "Internal error." },
  { Et::win_cln_al, "Alarm: The front panel of the safety laser scanner must be cleaned." },
  { Et::power_supply, "Power supply problem." },
  { Et::netw_prb, "Network problem." },
  { Et::dust_crc_fl, "Dust circuit failure" },
  { Et::ossd2_overcur, "OSSD2 Overcurrent / Short circuit." },
  { Et::meas_prob, "Measurement Problem." },
  { Et::incoherence, "Incoherence Error" },
  { Et::zone_inval_trans, "Zone: Invalid input - transition or integrity." },
  { Et::zone_invalid_conf, "Zone: Invalid input configuration / connection." },
  { Et::win_cln_warn, "Warning: The front panel of the safety laser scanner must be cleaned." },
  { Et::int_com_prb, "Internal communication problem." },
  { Et::generic_err, "Generic Error." },
  { Et::disp_com_prb, "Display communication problem." },
  { Et::temp_meas_prob, "Temperature measurement problem." },
  { Et::encod_oor, "Encoder: Out of range." },
  { Et::edm2_err, "Error in the External Device Monitoring (EDM2_ERR)." },
  { Et::edm1_err, "Error in the External Device Monitoring (EDM1_ERR)." },
  { Et::conf_err, "Configuration Error." },
  { Et::out_of_range_err, "Out of range error." },
  { Et::temp_range_err, "Temperature out of range." },
  { Et::encoder_generic_err, "Encoder: Generic error." },
  { Et::unused, "Unexpected error" } \
};

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<ErrorType, 8>, 9> error_bits{{
  //Bit7                 Bit6              Bit5              Bit4              Bit3              Bit2                  Bit1                   Bit0
  { REV(Et::ossd1_oc,    Et::ossd_shrt_c,  Et::ossd_integr,  Et::intern,       Et::intern,       Et::intern,           Et::intern,            Et::intern) },
  { REV(Et::win_cln_al,  Et::power_supply, Et::netw_prb,     Et::dust_crc_fl,  Et::intern,       Et::intern,           Et::unused,            Et::ossd2_overcur) },
  { REV(Et::meas_prob,   Et::intern,       Et::intern,       Et::intern,       Et::incoherence,  Et::zone_inval_trans, Et::zone_invalid_conf, Et::win_cln_warn) },
  { REV(Et::int_com_prb, Et::intern,       Et::intern,       Et::generic_err,  Et::disp_com_prb, Et::intern,           Et::intern,            Et::temp_meas_prob) },
  { REV(Et::encod_oor,   Et::unused,       Et::unused,       Et::edm2_err,     Et::edm1_err,     Et::conf_err,         Et::out_of_range_err,  Et::temp_range_err) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,            Et::encoder_generic_err) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,            Et::unused) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,            Et::unused) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,            Et::unused) },
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

class Message
{
public:
  constexpr Message(const ScannerId& id, const diagnostic::ErrorLocation& location);
  constexpr bool operator==(const diagnostic::Message& rhs) const;

  friend diagnostic::raw_message::Field diagnostic::serialize(const std::vector<diagnostic::Message>& messages);

  constexpr ScannerId getScannerId() const
  {
    return id_;
  }

  constexpr ErrorLocation getErrorLocation() const
  {
    return error_location_;
  }

  constexpr ErrorType getDiagnosticCode() const
  {
    return error_bits.at(error_location_.getByte()).at(error_location_.getBit());
  }

private:
  ScannerId id_;
  ErrorLocation error_location_;
};

constexpr inline Message::Message(const ScannerId& id, const ErrorLocation& location)
  : id_(id), error_location_(location)
{
}

constexpr inline bool Message::operator==(const Message& rhs) const
{
  return (error_location_.getBit() == rhs.error_location_.getBit() &&
          error_location_.getByte() == rhs.error_location_.getByte() && id_ == rhs.id_);
}

// Store ambiguous errors for additional output
static const std::set<Et> ambiguous_diagnostic_codes = { Et::unused, Et::intern };

inline bool isAmbiguous(const ErrorType& code)
{
  return ambiguous_diagnostic_codes.find(code) != ambiguous_diagnostic_codes.end();
}

std::ostream& operator<<(std::ostream& os, const diagnostic::Message& msg);

}  // namespace diagnostic
}  // namespace monitoring_frame
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_DIAGNOSTICS_H
