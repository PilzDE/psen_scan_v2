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

inline std::ostream& operator<<(std::ostream& os, const PinState& pin_state)
{
  return os << fmt::format(
             "PinState(id = {}, name = {}, state = {})", pin_state.id(), pin_state.name(), pin_state.state());
}

//! @brief Represents the set of all I/Os of the scanner and their states.
class IOState
{
public:
  IOState() = default;
  IOState(std::vector<PinState> physical_input_0,
          std::vector<PinState> physical_input_1,
          std::vector<PinState> physical_input_2,
          std::vector<PinState> logical_input,
          std::vector<PinState> output);
  std::vector<PinState> physicalInput0() const;
  std::vector<PinState> physicalInput1() const;
  std::vector<PinState> physicalInput2() const;
  std::vector<PinState> logicalInput() const;
  std::vector<PinState> output() const;

private:
  std::vector<PinState> physical_input_0_{};
  std::vector<PinState> physical_input_1_{};
  std::vector<PinState> physical_input_2_{};
  std::vector<PinState> logical_input_{};
  std::vector<PinState> output_{};
};

inline IOState::IOState(std::vector<PinState> physical_input_0,
                        std::vector<PinState> physical_input_1,
                        std::vector<PinState> physical_input_2,
                        std::vector<PinState> logical_input,
                        std::vector<PinState> output)
  : physical_input_0_(std::move(physical_input_0))
  , physical_input_1_(std::move(physical_input_1))
  , physical_input_2_(std::move(physical_input_2))
  , logical_input_(std::move(logical_input))
  , output_(std::move(output))
{
}

inline std::vector<PinState> IOState::physicalInput0() const
{
  return physical_input_0_;
}

inline std::vector<PinState> IOState::physicalInput1() const
{
  return physical_input_1_;
}

inline std::vector<PinState> IOState::physicalInput2() const
{
  return physical_input_2_;
}

inline std::vector<PinState> IOState::logicalInput() const
{
  return logical_input_;
}

inline std::vector<PinState> IOState::output() const
{
  return output_;
}

inline std::ostream& operator<<(std::ostream& os, const IOState& io_state)
{
  return os << fmt::format(
             "IOState(physicalInput0 = {}, physicalInput1 = {}, physicalInput2 = {}, logicalInput = {}, output = {})",
             util::formatRange(io_state.physicalInput0()),
             util::formatRange(io_state.physicalInput1()),
             util::formatRange(io_state.physicalInput2()),
             util::formatRange(io_state.logicalInput()),
             util::formatRange(io_state.output()));
}
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_STATE_H
