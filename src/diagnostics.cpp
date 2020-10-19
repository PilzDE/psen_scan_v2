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
MonitoringFrameDiagnosticMessage::MonitoringFrameDiagnosticMessage(ScannerId id, ErrorLocation location)
  : id_(id), code_(error_bits[location.getByte()][location.getBit()]), error_location_(location)
{
}

bool MonitoringFrameDiagnosticMessage::operator==(const MonitoringFrameDiagnosticMessage& rhs) const
{
  return (error_location_.getBit() == rhs.error_location_.getBit() &&
          error_location_.getByte() == rhs.error_location_.getByte() && code_ == rhs.code_ && id_ == rhs.id_);
}

std::ostream& operator<<(std::ostream& os, const MonitoringFrameDiagnosticMessage& msg)
{
  os << fmt::format(
      "Device: {} - {}", scanner_id_to_string.at(msg.getScannerId()), error_code_to_string.at(msg.getDiagnosticCode()));

  if (isAmbiguous(msg.getDiagnosticCode()))
  {
    os << fmt::format(" (Byte:{} Bit:{})", msg.getErrorLocation().getByte(), msg.getErrorLocation().getBit());
  }

  return os;
}

}  // namespace psen_scan_v2
