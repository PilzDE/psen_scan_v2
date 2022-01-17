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

#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_state_conversions.h"

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
  os << fmt::format("PinState(id = {}, name = {}, state = {})", pin_state.id(), pin_state.name(), pin_state.state());
  return os;
}

IOState::IOState(data_conversion_layer::monitoring_frame::io::PinData pin_data, const int64_t& timestamp)
  : pin_data_(pin_data), timestamp_(timestamp)
{
}

bool IOState::operator==(const IOState& io_state) const
{
  return pin_data_ == io_state.pin_data_;
}

bool IOState::operator!=(const IOState& io_state) const
{
  return !operator==(io_state);
}

std::vector<PinState> IOState::input() const
{
  return data_conversion_layer::generateInputPinStates(pin_data_);
}

std::vector<PinState> IOState::output() const
{
  return data_conversion_layer::generateOutputPinStates(pin_data_);
}

int64_t IOState::timestamp() const
{
  return timestamp_;
}

std::vector<PinState> IOState::changedInputStates(const IOState& ref_state) const
{
  return data_conversion_layer::generateChangedInputStates(pin_data_, ref_state.pin_data_);
}

std::vector<PinState> IOState::changedOutputStates(const IOState& ref_state) const
{
  return data_conversion_layer::generateChangedOutputStates(pin_data_, ref_state.pin_data_);
}

std::ostream& operator<<(std::ostream& os, const IOState& io_state)
{
  return os << "IOState(timestamp = " << io_state.timestamp_ << " nsec, " << io_state.pin_data_ << ")";
}
}  // namespace psen_scan_v2_standalone
