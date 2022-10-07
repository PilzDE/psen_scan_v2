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

#include <ostream>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
namespace diagnostic
{
std::ostream& operator<<(std::ostream& os, const Message& msg)
{
  os << fmt::format("Device: {} - {}",
                    configuration::SCANNER_ID_TO_STRING.at(msg.scannerId()),
                    ERROR_CODE_TO_STRING.at(msg.diagnosticCode()));

  if (isAmbiguous(msg.diagnosticCode()))
  {
    os << fmt::format(" (Byte:{} Bit:{})", msg.errorLocation().byte(), msg.errorLocation().bit());
  }

  return os;
}

}  // namespace diagnostic
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
