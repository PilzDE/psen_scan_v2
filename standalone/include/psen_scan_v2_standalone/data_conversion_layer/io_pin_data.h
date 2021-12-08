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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_PIN_H
#define PSEN_SCAN_V2_STANDALONE_IO_PIN_H

#include <functional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "psen_scan_v2_standalone/data_conversion_layer/io_constants.h"
#include "psen_scan_v2_standalone/io_state.h"

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
static uint32_t createID(size_t byte_n, size_t bit_n)
{
  return byte_n * 8 + bit_n;
}

typedef std::function<PinState(size_t, size_t, bool)> AddPinStateFunction;

inline PinState createLogicalPinState(size_t byte_n, size_t bit_n, bool value)
{
  if (byte_n >= RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES)
  {
    throw std::out_of_range("");
  }
  auto id = createID(byte_n, bit_n);
  auto input_bit = LOGICAL_INPUT_BITS.at(byte_n).at(bit_n);
  const auto& name = LOGICAL_INPUT_BIT_TO_NAME.at(input_bit);
  return PinState(id, name, value);
}

inline PinState createOutputPinState(size_t byte_n, size_t bit_n, bool value)
{
  if (byte_n >= RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES)
  {
    throw std::out_of_range("");
  }
  auto id = createID(byte_n, bit_n);
  auto output_bit = OUTPUT_BITS.at(byte_n).at(bit_n);
  const auto& name = OUTPUT_BIT_TO_NAME.at(output_bit);
  return PinState(id, name, value);
}

/**
 * @brief Represents the IO PIN field of a monitoring frame.
 */
struct PinData
{
  using States = std::vector<PinState>;
  States logical_input{};
  States output{};
};

inline std::ostream& operator<<(std::ostream& os, const PinData& pd)
{
  return os << fmt::format(
             "io::PinData(logical_input = {}, output = {})",
             util::formatRange(pd.logical_input),
             util::formatRange(pd.output));
}
}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_IO_PIN_H
