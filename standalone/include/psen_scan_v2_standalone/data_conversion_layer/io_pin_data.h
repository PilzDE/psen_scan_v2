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
  bool inputPinState(std::size_t byte_location, std::size_t bit_location) const;
  //! @throws std::out_of_range if byte_location >= NUMBER_OF_OUTPUT_BYTES or bit_location >= 8
  bool outputPinState(std::size_t byte_location, std::size_t bit_location) const;

  //! @throws std::out_of_range if byte_location >= NUMBER_OF_INPUT_BYTES or bit_location >= 8
  void inputPinState(std::size_t byte_location, std::size_t bit_location, bool pin_state);
  //! @throws std::out_of_range if byte_location >= NUMBER_OF_OUTPUT_BYTES or bit_location >= 8
  void outputPinState(std::size_t byte_location, std::size_t bit_location, bool pin_state);

  std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES>& inputState();
  std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES>& outputState();
  const std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES>& inputState() const;
  const std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES>& outputState() const;

  bool operator==(const PinData& pin_data) const;

private:
  std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES> input_states_{};
  std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES> output_states_{};
};

inline bool PinData::inputPinState(std::size_t byte_location, std::size_t bit_location) const
{
  return input_states_.at(byte_location).test(bit_location);
}

inline bool PinData::outputPinState(std::size_t byte_location, std::size_t bit_location) const
{
  return output_states_.at(byte_location).test(bit_location);
}

inline void PinData::inputPinState(std::size_t byte_location, std::size_t bit_location, bool pin_state)
{
  input_states_.at(byte_location).set(bit_location, pin_state);
}

inline void PinData::outputPinState(std::size_t byte_location, std::size_t bit_location, bool pin_state)
{
  output_states_.at(byte_location).set(bit_location, pin_state);
}

inline std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES>& PinData::inputState()
{
  return input_states_;
}

inline std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES>& PinData::outputState()
{
  return output_states_;
}

inline const std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES>& PinData::inputState() const
{
  return input_states_;
}

inline const std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES>& PinData::outputState() const
{
  return output_states_;
}

inline bool PinData::operator==(const PinData& pin_data) const
{
  return input_states_ == pin_data.input_states_ && output_states_ == pin_data.output_states_;
}

// LCOV_EXCL_START
inline std::ostream& operator<<(std::ostream& os, const PinData& pd)
{
  return os << fmt::format("io::PinData(input = {}, output = {})",
                           util::formatRange(pd.inputState()),
                           util::formatRange(pd.outputState()));
}
// LCOV_EXCL_STOP

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_IO_PIN_DATA_H
