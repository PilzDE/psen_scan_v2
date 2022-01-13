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
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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
inline static uint32_t createID(size_t byte_n, size_t bit_n)
{
  return byte_n * 8 + bit_n;
}

/**
 * @brief Represents the IO PIN field of a monitoring frame.
 */
struct PinData
{
  using States = std::vector<PinState>;
  States input{};
  States output{};
};

// LCOV_EXCL_START
inline std::ostream& operator<<(std::ostream& os, const PinData& pd)
{
  return os << fmt::format(
             "io::PinData(input = {}, output = {})", util::formatRange(pd.input), util::formatRange(pd.output));
}
// LCOV_EXCL_STOP

}  // namespace io
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_IO_PIN_H
