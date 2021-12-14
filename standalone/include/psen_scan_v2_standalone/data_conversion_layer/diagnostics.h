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

#ifndef PSEN_SCAN_V2_STANDALONE_DIAGNOSTICS_H
#define PSEN_SCAN_V2_STANDALONE_DIAGNOSTICS_H

#include <array>
#include <string>
#include <map>
#include <vector>
#include <set>

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"

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
 * @brief Contains all types, etc. needed to describe the diagnostics information contained
 * in a  data_conversion_layer::monitoring_frame::Message.
 */
namespace diagnostic
{
/**
 * @brief Contains constants and types needed to define the diagnostic::Message.
 */
static constexpr uint32_t RAW_CHUNK_LENGTH_FOR_ONE_DEVICE_IN_BYTES{ 9 };
static constexpr uint32_t RAW_CHUNK_UNUSED_OFFSET_IN_BYTES{ 4 };
static constexpr uint32_t RAW_CHUNK_LENGTH_IN_BYTES{
  RAW_CHUNK_UNUSED_OFFSET_IN_BYTES + RAW_CHUNK_LENGTH_FOR_ONE_DEVICE_IN_BYTES * configuration::VALID_SCANNER_IDS.size()
};

using RawChunk = std::array<uint8_t, diagnostic::RAW_CHUNK_LENGTH_IN_BYTES>;

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

static const std::map<ErrorType, ErrorMessage> ERROR_CODE_TO_STRING
{
  { Et::ossd1_oc, "OSSD1 Overcurrent / Short circuit." },
  { Et::ossd_shrt_c, "Short circuit between at least two OSSDs." },
  { Et::ossd_integr, "OSSDF1: An error has occurred when testing the OSSDs." },
  { Et::intern, "Internal error." },
  { Et::win_cln_al, "Alarm: The front panel of the safety laser scanner must be cleaned." },
  { Et::power_supply, "Power supply problem." },
  { Et::netw_prb, "Network problem." },
  { Et::dust_crc_fl, "Dust circuit failure" },
  { Et::ossd2_overcur, "OSSD2 Overcurrent / Short circuit." },
  { Et::meas_prob, "Measurement Problem." },
  { Et::incoherence, "Incoherence Error" },
  { Et::zone_inval_trans, "INPUTCF2: Configuration error. - "
                           "In the configuration, check the configured state transitions and switching operations." },
  { Et::zone_invalid_conf, "INPUTCF1: Error in the configuration or the wiring. - "
                           "Check whether the wiring and the configuration will match." },
  { Et::win_cln_warn, "Warning: The front panel of the safety laser scanner must be cleaned." },
  { Et::generic_err, "Generic Error." },
  { Et::disp_com_prb, "Display communication problem." },
  { Et::temp_meas_prob, "Temperature measurement problem." },
  { Et::encod_oor, "Encoder: Out of range." },
  { Et::edm2_err, "EDM2: Error in the External Device Monitoring." },
  { Et::edm1_err, "EDM1: Error in the External Device Monitoring." },
  { Et::conf_err, "WAITING_CONF: The safety laser scanner waits for a configuration (e.g. after restoring a configuration). - "
                  "Configure the safety laser scanner." },
  { Et::out_of_range_err, "Out of range error." },
  { Et::temp_range_err, "Temperature out of range." },
  { Et::encoder_generic_err, "Encoder: Generic error." },
  { Et::unused, "Unexpected error" } \
};

// clang-format off
  #define REV(arg1, arg2, arg3, arg4, arg5, arg6, arg7, arg8) arg8, arg7, arg6, arg5, arg4, arg3, arg2, arg1

  static constexpr std::array<std::array<ErrorType, 8>, 9> ERROR_BITS{{
  //Bit7                 Bit6              Bit5              Bit4              Bit3              Bit2                  Bit1                   Bit0
  { REV(Et::ossd1_oc,    Et::ossd_shrt_c,  Et::ossd_integr,  Et::intern,       Et::intern,       Et::intern,           Et::intern,              Et::intern) },
  { REV(Et::win_cln_al,  Et::power_supply, Et::netw_prb,     Et::dust_crc_fl,  Et::intern,       Et::intern,           Et::unused,              Et::ossd2_overcur) },
  { REV(Et::meas_prob,   Et::intern,       Et::intern,       Et::intern,       Et::incoherence,  Et::zone_inval_trans, Et::zone_invalid_conf,   Et::win_cln_warn) },
  { REV(Et::intern,      Et::intern,       Et::intern,       Et::generic_err,  Et::disp_com_prb, Et::intern,           Et::intern,              Et::temp_meas_prob) },
  { REV(Et::intern,      Et::intern,       Et::edm2_err,     Et::edm1_err,     Et::conf_err,     Et::out_of_range_err, Et::temp_range_err,      Et::intern) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::intern,       Et::intern,           Et::encoder_generic_err, Et::encod_oor) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,              Et::unused) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,              Et::unused) },
  { REV(Et::unused,      Et::unused,       Et::unused,       Et::unused,       Et::unused,       Et::unused,           Et::unused,              Et::unused) },
  }};
// clang-format on

/**
 * @brief Defines a byte and bit position of an error in the diagnostic chunk.
 *
 * The diagnostic chunk provided by the scanner when diagnostics are enabled in the ScannerConfiguration
 * is a set of error bits stored in consecutive bytes.
 *
 * This class helps defining the positions of those error bits.
 *
 * @see data_conversion_layer::monitoring_frame::diagnostic::error_bits
 * @see ScannerConfiguration
 */
class ErrorLocation
{
public:
  using ByteLocation = size_t;
  using BitLocation = size_t;
  constexpr ErrorLocation(const ByteLocation& byte, const BitLocation& bit) : byte_(byte), bit_(bit){};

  /*! deprecated: use inline constexpr ByteLocation byte() const instead */
  [[deprecated("use inline constexpr ByteLocation byte() const instead")]] inline constexpr ByteLocation getByte() const
  {
    return byte_;
  };

  inline constexpr ByteLocation byte() const
  {
    return byte_;
  };

  /*! deprecated: use inline constexpr BitLocation bit() const instead */
  [[deprecated("use inline constexpr BitLocation bit() const instead")]] inline constexpr BitLocation getBit() const
  {
    return bit_;
  };

  inline constexpr BitLocation bit() const
  {
    return bit_;
  };

private:
  ByteLocation byte_;
  BitLocation bit_;
};

/**
 * @brief Defines an Diagnostic message by defining the ErrorLocation and a scanner ID
 *
 * With the provided information a message can be generated for this specific diagnostic incident.
 * The Message object then can be used in an std::ostream to print the diagnostic message defined in the
 * `error_code_to_string` array.
 *
 * @see data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation
 * @see data_conversion_layer::monitoring_frame::diagnostic::error_bits
 * @see data_conversion_layer::monitoring_frame::diagnostic::error_code_to_string
 */
class Message
{
public:
  constexpr Message(const configuration::ScannerId& id, const diagnostic::ErrorLocation& location);
  constexpr bool operator==(const diagnostic::Message& rhs) const;

  friend RawChunk serialize(const std::vector<diagnostic::Message>& messages);

  /*! deprecated: use constexpr configuration::ScannerId scannerId() const instead */
  [[deprecated("use constexpr configuration::ScannerId scannerId() const instead")]] constexpr configuration::ScannerId
  getScannerId() const
  {
    return id_;
  }

  constexpr configuration::ScannerId scannerId() const
  {
    return id_;
  }

  /*! deprecated: use constexpr ErrorLocation errorLocation() const instead */
  [[deprecated("use constexpr ErrorLocation errorLocation() const instead")]] constexpr ErrorLocation
  getErrorLocation() const
  {
    return error_location_;
  }

  constexpr ErrorLocation errorLocation() const
  {
    return error_location_;
  }

  /*! deprecated: use constexpr ErrorType diagnosticCode() const instead */
  [[deprecated("use constexpr ErrorType diagnosticCode() const instead")]] constexpr ErrorType getDiagnosticCode() const
  {
    return ERROR_BITS.at(error_location_.byte()).at(error_location_.bit());
  }

  constexpr ErrorType diagnosticCode() const
  {
    return ERROR_BITS.at(error_location_.byte()).at(error_location_.bit());
  }

private:
  configuration::ScannerId id_;
  ErrorLocation error_location_;
};

constexpr inline Message::Message(const configuration::ScannerId& id, const ErrorLocation& location)
  : id_(id), error_location_(location)
{
}

constexpr inline bool Message::operator==(const Message& rhs) const
{
  return (error_location_.bit() == rhs.error_location_.bit() && error_location_.byte() == rhs.error_location_.byte() &&
          id_ == rhs.id_);
}

// Store ambiguous errors for additional output
static const std::set<Et> AMBIGUOUS_DIAGNOSTIC_CODES = { Et::unused, Et::intern };

inline bool isAmbiguous(const ErrorType& code)
{
  return AMBIGUOUS_DIAGNOSTIC_CODES.find(code) != AMBIGUOUS_DIAGNOSTIC_CODES.end();
}

std::ostream& operator<<(std::ostream& os, const diagnostic::Message& msg);

}  // namespace diagnostic
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_DIAGNOSTICS_H
