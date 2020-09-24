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
#ifndef PSEN_SCAN_V2_RAW_PROCESSING_H
#define PSEN_SCAN_V2_RAW_PROCESSING_H

#include <sstream>

#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/string_stream_failure.h"

namespace psen_scan_v2
{
namespace raw_processing
{
template <typename T>
inline void write(std::ostringstream& os, const T& data)
{
  os.write(reinterpret_cast<const char*>(&data), sizeof(T));
}

template <typename T>
inline void read(std::istringstream& is, T& data)
{
  is.read(reinterpret_cast<char*>(&data), sizeof(T));
  if (!is)
  {
    throw StringStreamFailure("Reading from string stream failed.");
  }
}
}  // namespace raw_processing
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_RAW_PROCESSING_H
