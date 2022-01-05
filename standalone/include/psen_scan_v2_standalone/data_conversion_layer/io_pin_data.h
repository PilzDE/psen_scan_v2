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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_PIN_DATA_H
#define PSEN_SCAN_V2_STANDALONE_IO_PIN_DATA_H

#include <array>
#include <bitset>
#include <ostream>
#include <sstream>
#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/data_conversion_layer/io_constants.h"
#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/util/format_range.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
/**
 * @brief Contains all types, etc. needed to describe the IOs information contained
 * in a  data_conversion_layer::monitoring_frame::Message.
 */
namespace io
{
static constexpr uint32_t NUMBER_OF_INPUT_BYTES{ RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES };
static constexpr uint32_t NUMBER_OF_OUTPUT_BYTES{ RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES };

//! @throws std::out_of_range if byte_location >= NUMBER_OF_INPUT_BYTES or bit_location >= 8
static inline LogicalInputType getInputType(std::size_t byte_location, std::size_t bit_location)
{
  return LOGICAL_INPUT_BITS.at(byte_location).at(bit_location);
}

//! @throws std::out_of_range if byte_location >= NUMBER_OF_INPUT_BYTES or bit_location >= 8
static inline std::string getInputName(std::size_t byte_location, std::size_t bit_location)
{
  return LOGICAL_INPUT_BIT_TO_NAME.at(getInputType(byte_location, bit_location));
}

//! @throws std::out_of_range if byte_location >= NUMBER_OF_OUTPUT_BYTES or bit_location >= 8
static inline OutputType getOutputType(std::size_t byte_location, std::size_t bit_location)
{
  return OUTPUT_BITS.at(byte_location).at(bit_location);
}

//! @throws std::out_of_range if byte_location >= NUMBER_OF_OUTPUT_BYTES or bit_location >= 8
static inline std::string getOutputName(std::size_t byte_location, std::size_t bit_location)
{
  return OUTPUT_BIT_TO_NAME.at(getOutputType(byte_location, bit_location));
}

/**
 * @brief Represents the IO PIN field of a monitoring frame.
 */
class PinData
{
public:
  //! @throws std::out_of_range if byte_location >= NUMBER_OF_INPUT_BYTES or bit_location >= 8
  bool getInputState(std::size_t byte_location, std::size_t bit_location) const;
  //! @throws std::out_of_range if byte_location >= NUMBER_OF_OUTPUT_BYTES or bit_location >= 8
  bool getOutputState(std::size_t byte_location, std::size_t bit_location) const;

  //! @throws std::out_of_range if byte_location >= NUMBER_OF_INPUT_BYTES or bit_location >= 8
  void setInputState(std::size_t byte_location, std::size_t bit_location, bool pin_state);
  //! @throws std::out_of_range if byte_location >= NUMBER_OF_OUTPUT_BYTES or bit_location >= 8
  void setOutputState(std::size_t byte_location, std::size_t bit_location, bool pin_state);

public:
  std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES> input_states_{};
  std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES> output_states_{};
};

inline bool PinData::getInputState(std::size_t byte_location, std::size_t bit_location) const
{
  input_states_.at(byte_location).test(bit_location);
  return input_states_.at(byte_location)[bit_location];
}

inline bool PinData::getOutputState(std::size_t byte_location, std::size_t bit_location) const
{
  output_states_.at(byte_location).test(bit_location);
  return output_states_.at(byte_location)[bit_location];
}

inline void PinData::setInputState(std::size_t byte_location, std::size_t bit_location, bool pin_state)
{
  input_states_.at(byte_location).set(bit_location, pin_state);
}

inline void PinData::setOutputState(std::size_t byte_location, std::size_t bit_location, bool pin_state)
{
  output_states_.at(byte_location).set(bit_location, pin_state);
}

// LCOV_EXCL_START
inline std::ostream& operator<<(std::ostream& os, const PinData& pd)
{
  return os;
  // todo
  // return os << fmt::format("io::PinData(input_states_ = {}, output_states_ = {})",
  //                          util::formatRange(pd.input_states_),
  //                          util::formatRange(pd.output_states_));
}
// LCOV_EXCL_STOP

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_IO_PIN_DATA_H
