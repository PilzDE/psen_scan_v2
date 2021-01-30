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
#ifndef PSEN_SCAN_V2_RAW_DATA_ARRAY_CONVERSION_H
#define PSEN_SCAN_V2_RAW_DATA_ARRAY_CONVERSION_H

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

namespace psen_scan_v2_standalone_test
{
template <typename T>
inline psen_scan_v2_standalone::data_conversion_layer::RawData convertToRawData(const T data)
{
  psen_scan_v2_standalone::data_conversion_layer::RawData ret(data.begin(), data.end());

  for (size_t i = 0; i < data.size(); i++)
  {
    ret.at(i) = static_cast<char>(data.at(i));
  }
  return ret;
}
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_RAW_DATA_ARRAY_CONVERSION_H
