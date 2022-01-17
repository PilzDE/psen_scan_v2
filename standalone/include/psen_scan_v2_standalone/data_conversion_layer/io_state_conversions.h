// Copyright (c) 2022 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_STATE_CONVERSIONS_H
#define PSEN_SCAN_V2_STANDALONE_IO_STATE_CONVERSIONS_H

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"

#include <vector>

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
static inline PinState generateInputPinState(const monitoring_frame::io::PinData& pin_data,
                                             std::size_t byte_location,
                                             std::size_t bit_location)
{
  return { static_cast<uint32_t>(byte_location * 8 + bit_location),
           monitoring_frame::io::getInputName(byte_location, bit_location),
           pin_data.input_state.at(byte_location).test(bit_location) };
}

static inline PinState generateOutputPinState(const monitoring_frame::io::PinData& pin_data,
                                              std::size_t byte_location,
                                              std::size_t bit_location)
{
  return { static_cast<uint32_t>(byte_location * 8 + bit_location),
           monitoring_frame::io::getOutputName(byte_location, bit_location),
           pin_data.output_state.at(byte_location).test(bit_location) };
}

static inline bool isUsedInputBit(std::size_t byte_n, std::size_t bit_n)
{
  return data_conversion_layer::monitoring_frame::io::getInputType(byte_n, bit_n) !=
         data_conversion_layer::monitoring_frame::io::LogicalInputType::unused;
}

static inline bool isUsedOutputBit(std::size_t byte_n, std::size_t bit_n)
{
  return data_conversion_layer::monitoring_frame::io::getOutputType(byte_n, bit_n) !=
         data_conversion_layer::monitoring_frame::io::OutputType::unused;
}

static inline std::vector<PinState> generateInputPinStates(const monitoring_frame::io::PinData& pin_data)
{
  std::vector<PinState> pin_states;
  for (std::size_t byte_n = 0; byte_n < monitoring_frame::io::NUMBER_OF_INPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      if (isUsedInputBit(byte_n, bit_n))
      {
        pin_states.emplace_back(generateInputPinState(pin_data, byte_n, bit_n));
      }
    }
  }
  return pin_states;
}

static inline std::vector<PinState> generateOutputPinStates(const monitoring_frame::io::PinData& pin_data)
{
  std::vector<PinState> pin_states;
  for (std::size_t byte_n = 0; byte_n < monitoring_frame::io::NUMBER_OF_OUTPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      if (isUsedOutputBit(byte_n, bit_n))
      {
        pin_states.emplace_back(generateOutputPinState(pin_data, byte_n, bit_n));
      }
    }
  }
  return pin_states;
}

static inline std::vector<PinState> generateChangedInputStates(const monitoring_frame::io::PinData& new_state,
                                                               const monitoring_frame::io::PinData& old_state)
{
  std::vector<PinState> pin_states;
  for (std::size_t byte_n = 0; byte_n < new_state.input_state.size(); ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      if (new_state.input_state.at(byte_n).test(bit_n) ^ old_state.input_state.at(byte_n).test(bit_n))
      {
        pin_states.emplace_back(generateInputPinState(new_state, byte_n, bit_n));
      }
    }
  }
  return pin_states;
}

static inline std::vector<PinState> generateChangedOutputStates(const monitoring_frame::io::PinData& new_state,
                                                                const monitoring_frame::io::PinData& old_state)
{
  std::vector<PinState> pin_states;
  for (std::size_t byte_n = 0; byte_n < new_state.output_state.size(); ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      if (new_state.output_state.at(byte_n).test(bit_n) ^ old_state.output_state.at(byte_n).test(bit_n))
      {
        pin_states.emplace_back(generateOutputPinState(new_state, byte_n, bit_n));
      }
    }
  }
  return pin_states;
}

}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_STATE_CONVERSIONS_H
