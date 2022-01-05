// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
using namespace data_conversion_layer::monitoring_frame::io;

template <typename PinType, size_t ChunkSize>
inline std::vector<PinState>
createCompletePinField(const std::array<std::array<PinType, 8>, ChunkSize>& type_lookup_array,
                       const std::map<PinType, std::string>& name_lookup_map)
{
  std::vector<PinState> pin_field;
  for (std::size_t byte = 0; byte < ChunkSize; ++byte)
  {
    for (std::size_t bit = 0; bit < 8; ++bit)
    {
      auto pin_type = type_lookup_array.at(byte).at(bit);
      if (pin_type != PinType::unused)
      {
        pin_field.emplace_back(createID(byte, bit), name_lookup_map.at(pin_type), false);
      }
    }
  }
  return pin_field;
}

inline PinData createCompleteIOPinData()
{
  PinData pin_data;
  pin_data.logical_input = createCompletePinField(LOGICAL_INPUT_BITS, LOGICAL_INPUT_BIT_TO_NAME);
  pin_data.output = createCompletePinField(OUTPUT_BITS, OUTPUT_BIT_TO_NAME);
  return pin_data;
}

inline void setPin(PinState& pin_state)
{
  pin_state = PinState(pin_state.id(), pin_state.name(), true);
}

inline void setInputPin(PinData& io_pin_data, const LogicalInputType& type)
{
  const auto it = std::find_if(io_pin_data.logical_input.begin(), io_pin_data.logical_input.end(), [&](const auto& a) {
    return a.name() == LOGICAL_INPUT_BIT_TO_NAME.at(type);
  });
  if (it == io_pin_data.logical_input.end())
  {
    throw std::invalid_argument("Could not set input pin because type " + LOGICAL_INPUT_BIT_TO_NAME.at(type) +
                                " is not included in the pin data.");
  }
  setPin(*it);
}

inline void setOutputPin(PinData& io_pin_data, const OutputType& type)
{
  const auto it = std::find_if(io_pin_data.output.begin(), io_pin_data.output.end(), [&](const auto& a) {
    return a.name() == OUTPUT_BIT_TO_NAME.at(type);
  });
  if (it == io_pin_data.output.end())
  {
    throw std::invalid_argument("Could not set output pin because type " + OUTPUT_BIT_TO_NAME.at(type) +
                                " is not included in the pin data.");
  }
  setPin(*it);
}
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_IO_PIN_DATA_HELPER_H
