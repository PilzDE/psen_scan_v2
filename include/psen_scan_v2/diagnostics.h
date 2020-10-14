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

#include <fmt/core.h>

#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/scanner_ids.h"

namespace psen_scan_v2
{
enum class ScannerId : uint8_t;

static constexpr uint32_t DIAGNOSTIC_MESSAGE_RAW_LENGTH_FOR_ONE_DEVICE_IN_BYTES{ 9 };
static constexpr uint32_t DIAGNOSTIC_MESSAGE_RAW_UNUSED_DATA_OFFSET_IN_BYTES{ 4 };
static constexpr uint32_t DIAGNOSTIC_DATA_FIELD_IN_MONITORING_FRAME_LENGTH_IN_BYTES{
  DIAGNOSTIC_MESSAGE_RAW_UNUSED_DATA_OFFSET_IN_BYTES +
  DIAGNOSTIC_MESSAGE_RAW_LENGTH_FOR_ONE_DEVICE_IN_BYTES * sizeof(SCANNER_IDS)
};

enum class DiagnosticCode;

enum class MonitoringFrameDiagnosticMessageLevel : uint8_t
{
  INFO,
  WARNING,
  DEBUG,
  ERROR
};

class MonitoringFrameDiagnosticMessage
{
public:
  using DiagnoseFieldErrorBitLocation = uint8_t;
  using DiagnoseFieldErrorByteLocation = uint16_t;

  MonitoringFrameDiagnosticMessage(ScannerId id,
                                   DiagnoseFieldErrorByteLocation byte_location,
                                   DiagnoseFieldErrorBitLocation bit_location);

  friend std::ostream& operator<<(std::ostream& os, const MonitoringFrameDiagnosticMessage& msg);
  friend std::array<uint8_t, DIAGNOSTIC_DATA_FIELD_IN_MONITORING_FRAME_LENGTH_IN_BYTES>
  serializeDiagnosticMessages(std::vector<MonitoringFrameDiagnosticMessage>& messages);

private:
  ScannerId id_;
  DiagnosticCode code_;
  DiagnoseFieldErrorByteLocation byte_location_;
  DiagnoseFieldErrorBitLocation bit_location_;
  MonitoringFrameDiagnosticMessageLevel level_;
};

// clang-format off
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

  using ErrorCodeName = std::string;

  extern std::map<DiagnosticCode, ErrorCodeName> error_code_to_string;
  extern std::array<std::array<DiagnosticCode, 8>, 9> error_bits;

}  //  namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_DIAGNOSTICS_H