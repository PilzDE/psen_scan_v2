// Copyright (c) 2021-2022 Pilz GmbH & Co. KG
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
#include <string>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/data_conversion_layer/io_constants.h"
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
struct PinData
{
  bool operator==(const PinData& pin_data) const;
  std::array<std::bitset<8>, NUMBER_OF_INPUT_BYTES> input_state{};
  std::array<std::bitset<8>, NUMBER_OF_OUTPUT_BYTES> output_state{};
};

inline bool PinData::operator==(const PinData& pin_data) const
{
  return input_state == pin_data.input_state && output_state == pin_data.output_state;
}

inline std::ostream& operator<<(std::ostream& os, const PinData& pd)
{
  return os << fmt::format("io::PinData(input = {}, output = {})",
                           util::formatRange(pd.input_state),
                           util::formatRange(pd.output_state));
}

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_IO_PIN_DATA_H
