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

bool MonitoringFrameDiagnosticMessage::operator==(const MonitoringFrameDiagnosticMessage& rhs) const
{
  return (bit_location_ == rhs.bit_location_ && byte_location_ == rhs.byte_location_ && code_ == rhs.code_ &&
          id_ == rhs.id_ && level_ == rhs.level_);
}

std::ostream& operator<<(std::ostream& os, const MonitoringFrameDiagnosticMessage& msg)
{
  // TODO ID to name (master, slave, ...)
  os << fmt::format(
      "id:{} {} (Byte: {} Bit:{})", msg.id_, error_code_to_string.at(msg.code_), msg.byte_location_, msg.bit_location_);
  return os;
}

}  // namespace psen_scan_v2