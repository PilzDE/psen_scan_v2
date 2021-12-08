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
#define PSEN_SCAN_V2_STAND

#include <string>
#include <vector>
#include <algorithm>

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
namespace dmi = data_conversion_layer::monitoring_frame::io;

inline std::vector<PinState> createPinField(uint32_t count, const dmi::AddPinStateFunction& add_func)
{
  std::vector<PinState> io_field;
  for (std::size_t byte = 0; byte < count; ++byte)
  {
    for (std::size_t bit = 0; bit < 8; ++bit)
    {
      const auto pin_state = add_func(byte, bit, false);
      if (pin_state.name() != "unused")
      {
        io_field.push_back(pin_state);
      }
    }
  }
  return io_field;
}

inline dmi::PinData createCompleteIOPinData()
{
  dmi::PinData pin_data;
  pin_data.logical_input = createPinField(dmi::RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES, dmi::createLogicalPinState);
  pin_data.output = createPinField(dmi::RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES, dmi::createOutputPinState);
  return pin_data;
}

inline void setPin(PinState& pin_state)
{
  pin_state = PinState(pin_state.id(), pin_state.name(), true);
}

inline void setInputPin(dmi::PinData& io_pin_data, const dmi::LogicalInputType& type)
{
  setPin(*std::find_if(io_pin_data.logical_input.begin(), io_pin_data.logical_input.end(), [&](const auto& a) {
    return a.name() == dmi::LOGICAL_INPUT_BIT_TO_NAME.at(type);
  }));
}

inline void setOutputPin(dmi::PinData& io_pin_data, const dmi::OutputType& type)
{
  setPin(*std::find_if(io_pin_data.output.begin(), io_pin_data.output.end(), [&](const auto& a) {
    return a.name() == dmi::OUTPUT_BIT_TO_NAME.at(type);
  }));
}
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_IO_PIN_DATA_HELPER_H