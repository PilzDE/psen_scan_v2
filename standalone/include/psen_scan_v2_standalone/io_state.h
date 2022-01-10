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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_STATE_H
#define PSEN_SCAN_V2_STANDALONE_IO_STATE_H

#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/util/format_range.h"

namespace psen_scan_v2_standalone
{
//! @brief Represents a single I/O pin.
class PinState
{
public:
  /**
   * @param pin_id Unique id of the pin inside an I/O record.
   * @param name Name connected to the pin.
   * @param state Binary state of the pin.
   */
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

// LCOV_EXCL_START
inline std::ostream& operator<<(std::ostream& os, const PinState& pin_state)
{
  return os << fmt::format(
             "PinState(id = {}, name = {}, state = {})", pin_state.id(), pin_state.name(), pin_state.state());
}
// LCOV_EXCL_STOP

//! @brief Represents the set of all I/Os of the scanner and their states.
class IOState
{
public:
  IOState() = default;
  IOState(std::vector<PinState> input, std::vector<PinState> output);
  const std::vector<PinState>& input() const;
  const std::vector<PinState>& output() const;

private:
  std::vector<PinState> input_{};
  std::vector<PinState> output_{};
};

inline IOState::IOState(std::vector<PinState> input, std::vector<PinState> output)
  : input_(std::move(input)), output_(std::move(output))
{
}

inline const std::vector<PinState>& IOState::input() const
{
  return input_;
}

inline const std::vector<PinState>& IOState::output() const
{
  return output_;
}

inline std::ostream& operator<<(std::ostream& os, const IOState& io_state)
{
  return os << fmt::format("IOState(input = {}, output = {})",
                           util::formatRange(io_state.input()),
                           util::formatRange(io_state.output()));
}
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_STATE_H
