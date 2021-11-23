// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_STATUS_H
#define PSEN_SCAN_V2_STANDALONE_IO_STATUS_H

#include <string>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/util/format_range.h"

namespace psen_scan_v2_standalone
{
class PinState
{
public:
  PinState(uint32_t pin_id, const std::string& name, bool state);
  bool operator==(const PinState& ps) const;
  bool operator!=(const PinState& ps) const;
  uint32_t id() const;
  std::string name() const;
  bool state() const;

private:
  uint32_t id_;
  std::string name_;
  bool state_;
};

inline PinState::PinState(uint32_t pin_id, const std::string& name, bool state)
  : id_(pin_id), name_(name), state_(state)
{
}

inline bool PinState::operator==(const PinState& ps) const
{
  return id() == ps.id() && name() == ps.name() && state() == ps.state();
}

inline bool PinState::operator!=(const PinState& ps) const
{
  return !operator==(ps);
}

inline uint32_t PinState::id() const
{
  return id_;
}

inline std::string PinState::name() const
{
  return name_;
}

inline bool PinState::state() const
{
  return state_;
}

inline std::ostream& operator<<(std::ostream& os, const PinState& pin_state)
{
  return os << fmt::format(
             "PinState(id = {}, name = {}, state = {})", pin_state.id(), pin_state.name(), pin_state.state());
}

class IOState
{
public:
  IOState() = default;
  IOState(std::vector<PinState> input_states,
          std::vector<PinState> output_states,
          std::vector<PinState> logical_states);
  std::vector<PinState> input() const;
  std::vector<PinState> output() const;
  std::vector<PinState> logical() const;

private:
  std::vector<PinState> input_;
  std::vector<PinState> output_;
  std::vector<PinState> logical_;
};

inline IOState::IOState(std::vector<PinState> input_states,
                        std::vector<PinState> output_states,
                        std::vector<PinState> logical_states)
  : input_(std::move(input_states)), output_(std::move(output_states)), logical_(std::move(logical_states))
{
}

inline std::vector<PinState> IOState::input() const
{
  return input_;
}

inline std::vector<PinState> IOState::output() const
{
  return output_;
}

inline std::vector<PinState> IOState::logical() const
{
  return logical_;
}

inline std::ostream& operator<<(std::ostream& os, const IOState& io_state)
{
  return os << fmt::format("IOState(input = {}, output = {}, logical = {})",
                           util::formatRange(io_state.input()),
                           util::formatRange(io_state.output()),
                           util::formatRange(io_state.logical()));
}
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_STATUS_H