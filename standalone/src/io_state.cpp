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

#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/util/format_range.h"

namespace psen_scan_v2_standalone
{
PinState::PinState(uint32_t pin_id, const std::string& name, bool state) : id_(pin_id), name_(name), state_(state)
{
}

bool PinState::operator==(const PinState& ps) const
{
  return id() == ps.id() && name() == ps.name() && state() == ps.state();
}

bool PinState::operator!=(const PinState& ps) const
{
  return !operator==(ps);
}

uint32_t PinState::id() const
{
  return id_;
}

std::string PinState::name() const
{
  return name_;
}

bool PinState::state() const
{
  return state_;
}

std::ostream& operator<<(std::ostream& os, const PinState& pin_state)
{
  return os << fmt::format(  // LCOV_EXCL_LINE lcov bug?
             "PinState(id = {}, name = {}, state = {})",
             pin_state.id(),
             pin_state.name(),
             pin_state.state());
}

IOState::IOState(std::vector<PinState> input, std::vector<PinState> output)
  : input_(std::move(input)), output_(std::move(output))
{
}

const std::vector<PinState>& IOState::input() const
{
  return input_;
}

const std::vector<PinState>& IOState::output() const
{
  return output_;
}

std::ostream& operator<<(std::ostream& os, const IOState& io_state)
{
  return os << fmt::format("IOState(input = {}, output = {})",
                           util::formatRange(io_state.input()),
                           util::formatRange(io_state.output()));
}
}  // namespace psen_scan_v2_standalone
