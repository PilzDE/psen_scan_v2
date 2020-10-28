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
  UNUSED
};

// clang-format off

using Et = ErrorType;
using ErrorMessage = std::string;

static const std::map<ErrorType, ErrorMessage> error_code_to_string
{
  { Et::OSSD1_OC, "OSSD1 Overcurrent / Short circuit." },
  { Et::OSSD_SHRT_C, "Short circuit between at least two OSSDs." },
  { Et::OSSD_INTEGR, "Integrity check problem on any OSSD" },
  { Et::INT, "Internal error." },
  { Et::WIN_CLN_AL, "Alarm: The front panel of the safety laser scanner must be cleaned." },
  { Et::POWER_SUPPLY, "Power supply problem." },
  { Et::NETW_PRB, "Network problem." },
  { Et::DUST_CRC_FL, "Dust circuit failure" },
  { Et::OSSD2_OVERCUR, "OSSD2 Overcurrent / Short circuit." },
  { Et::MEAS_PROB, "Measurement Problem." },
  { Et::INCOHERENCE, "Incoherence Error" },
  { Et::ZONE_INVAL_TRANS, "Zone: Invalid input - transition or integrity." },
  { Et::ZONE_INVALID_CONF, "Zone: Invalid input configuration / connection." },
  { Et::WIN_CLN_WARN, "Warning: The front panel of the safety laser scanner must be cleaned." },
  { Et::INT_COM_PRB, "Internal communication problem." },
  { Et::GENERIC_ERR, "Generic Error." },
  { Et::DISP_COM_PRB, "Display communication problem." },
  { Et::TEMP_MEAS_PROB, "Temperature measurement problem." },
  { Et::ENCOD_OOR, "Encoder: Out of range." },
  { Et::EDM2_ERR, "Error in the External Device Monitoring (EDM2_ERR)." },
  { Et::EDM1_ERR, "Error in the External Device Monitoring (EDM1_ERR)." },
  { Et::CONF_ERR, "Configuration Error." },
  { Et::OUT_OF_RANGE_ERR, "Out of range error." },
  { Et::TEMP_RANGE_ERR, "Temperature out of range." },
  { Et::ENCODER_GENERIC_ERR, "Encoder: Generic error." },
  { Et::UNUSED, "Unexpected error" } \
};

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<ErrorType, 8>, 9> error_bits{{
  //Bit7                 Bit6              Bit5              Bit4              Bit3              Bit2                  Bit1                   Bit0
  { REV(Et::OSSD1_OC,    Et::OSSD_SHRT_C,  Et::OSSD_INTEGR,  Et::INT,          Et::INT,          Et::INT,              Et::INT,               Et::INT) },
  { REV(Et::WIN_CLN_AL,  Et::POWER_SUPPLY, Et::NETW_PRB,     Et::DUST_CRC_FL,  Et::INT,          Et::INT,              Et::UNUSED,            Et::OSSD2_OVERCUR) },
  { REV(Et::MEAS_PROB,   Et::INT,          Et::INT,          Et::INT,          Et::INCOHERENCE,  Et::ZONE_INVAL_TRANS, Et::ZONE_INVALID_CONF, Et::WIN_CLN_WARN) },
  { REV(Et::INT_COM_PRB, Et::INT,          Et::INT,          Et::GENERIC_ERR,  Et::DISP_COM_PRB, Et::INT,              Et::INT,               Et::TEMP_MEAS_PROB) },
  { REV(Et::ENCOD_OOR,   Et::UNUSED,       Et::UNUSED,       Et::EDM2_ERR,     Et::EDM1_ERR,     Et::CONF_ERR,         Et::OUT_OF_RANGE_ERR,  Et::TEMP_RANGE_ERR) },
  { REV(Et::UNUSED,      Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,           Et::UNUSED,            Et::ENCODER_GENERIC_ERR) },
  { REV(Et::UNUSED,      Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,           Et::UNUSED,            Et::UNUSED) },
  { REV(Et::UNUSED,      Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,           Et::UNUSED,            Et::UNUSED) },
  { REV(Et::UNUSED,      Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,       Et::UNUSED,           Et::UNUSED,            Et::UNUSED) },
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
static const std::set<Et> ambiguous_diagnostic_codes = { Et::UNUSED, Et::INT };

inline bool isAmbiguous(const ErrorType& code)
{
  return ambiguous_diagnostic_codes.find(code) != ambiguous_diagnostic_codes.end();
}

std::ostream& operator<<(std::ostream& os, const diagnostic::Message& msg);

}  // namespace diagnostic
}  // namespace monitoring_frame
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_DIAGNOSTICS_H
