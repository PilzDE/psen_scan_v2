// Copyright (c) 2019-2022 Pilz GmbH & Co. KG
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

using psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io::NUMBER_OF_INPUT_BYTES;
using psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io::NUMBER_OF_OUTPUT_BYTES;
using psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io::PinData;

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

inline void setInputBit(PinData& pin_data, uint32_t id)
{
  pin_data.input_state.at(idToByte(id)).set(idToBit(id));
}

inline void setOutputBit(PinData& pin_data, uint32_t id)
{
  pin_data.output_state.at(idToByte(id)).set(idToBit(id));
}

inline PinData
createPinData(const std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES>& input_states = { 77, 0, 0, 0, 154, 0, 0 },
              const std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES>& output_states = { 85, 0, 0, 0 })
{
  PinData pin_data{};
  pin_data.input_state = input_states;
  pin_data.output_state = output_states;
  return pin_data;
}
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_IO_PIN_DATA_HELPER_H
