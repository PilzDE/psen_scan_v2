// Copyright (c) 2019-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_IO_PIN_DATA_HELPER_H
#define PSEN_SCAN_V2_STANDALONE_TEST_IO_PIN_DATA_HELPER_H

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"

namespace psen_scan_v2_standalone_test
{
inline std::size_t idToByte(uint32_t id)
{
  return static_cast<std::size_t>(id / 8);
}

inline std::size_t idToBit(uint32_t id)
{
  return static_cast<std::size_t>(id % 8);
}

inline void setInputBit(psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io::PinData& pin_data,
                        uint32_t id)
{
  return pin_data.inputPinState(idToByte(id), idToBit(id), true);
}

inline void setOutputBit(psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io::PinData& pin_data,
                         uint32_t id)
{
  return pin_data.outputPinState(idToByte(id), idToBit(id), true);
}

}  // namespace psen_scan_v2_standalone_test
#endif  // PSEN_SCAN_V2_STANDALONE_TEST_IO_PIN_DATA_HELPER_H