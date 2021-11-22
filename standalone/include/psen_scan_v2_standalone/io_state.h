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

namespace psen_scan_v2_standalone
{
class PinState
{
public:
  PinState(uint32_t pin_id, const std::string& name, bool state);
  uint32_t pinId() const;
  std::string name() const;
  bool state() const;

private:
  uint32_t pin_id_;
  std::string name_;
  bool state_;
};

inline PinState::PinState(uint32_t pin_id, const std::string& name, bool state) : pin_id_(pin_id), name_(name), state_(state)
{
}

inline uint32_t PinState::pinId() const
{
  return pin_id_;
}

inline std::string PinState::name() const
{
  return name_;
}

inline bool PinState::state() const
{
  return state_;
}

class IOState
{
public:
  IOState() = default;
  IOState(std::vector<PinState> input_states, std::vector<PinState> output_states, std::vector<PinState> logical_states);
  std::vector<PinState> input() const;
  std::vector<PinState> output() const;
  std::vector<PinState> logical() const;

private:
  std::vector<PinState> input_;
  std::vector<PinState> output_;
  std::vector<PinState> logical_;
};

inline IOState::IOState(std::vector<PinState> input_states, std::vector<PinState> output_states, std::vector<PinState> logical_states)
  : input_(input_states), output_(output_states), logical_(logical_states)
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
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_STATUS_H