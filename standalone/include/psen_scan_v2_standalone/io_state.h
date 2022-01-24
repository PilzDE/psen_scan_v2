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

#ifndef PSEN_SCAN_V2_STANDALONE_IO_STATE_H
#define PSEN_SCAN_V2_STANDALONE_IO_STATE_H

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"

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

std::ostream& operator<<(std::ostream& os, const PinState& pin_state);

//! @brief Formats a PinState just using its name and state. (e.g. Safety 1 intrusion = true)
inline std::string formatPinState(const PinState& pin)
{
  return fmt::format("{} = {}", pin.name(), pin.state());
}

//! @brief Formats a vector of PinStates
inline std::string formatPinStates(const std::vector<PinState>& pins)
{
  std::stringstream strstr;
  strstr << "{";
  for (auto it = pins.begin(); std::next(it) < pins.end(); ++it)
  {
    strstr << formatPinState(*it) << ", ";
  }
  if (!pins.empty())
  {
    strstr << formatPinState(pins.back());
  }
  strstr << "}";
  return strstr.str();
}

//! @brief Represents the set of all I/Os of the scanner and their states.
class IOState
{
public:
  IOState() = default;
  IOState(data_conversion_layer::monitoring_frame::io::PinData pin_data, const int64_t& timestamp);
  //! @return std::vector<PinState> containing a PinState for every (logical) input pin of the scanner.
  std::vector<PinState> input() const;
  //! @return std::vector<PinState> containing a PinState for every output pin of the scanner.
  std::vector<PinState> output() const;
  //! @return time[ns] of the monitoring frame this state is linked to.
  int64_t timestamp() const;
  /**
   * @param ref_state another IOState that is used as reference for the changed state calculation.
   * @return std::vector<PinState> containing a PinState for every changed input pin.
   */
  std::vector<PinState> changedInputStates(const IOState& ref_state) const;
  /**
   * @param ref_state another IOState that is used as reference for the changed state calculation.
   * @return std::vector<PinState> containing a PinState for every changed output pin.
   */
  std::vector<PinState> changedOutputStates(const IOState& ref_state) const;

  bool operator==(const IOState& io_state) const;
  bool operator!=(const IOState& io_state) const;

public:
  friend std::ostream& operator<<(std::ostream& os, const IOState& io_state);

private:
  data_conversion_layer::monitoring_frame::io::PinData pin_data_{};
  int64_t timestamp_{};
};

std::ostream& operator<<(std::ostream& os, const IOState& io_state);
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_IO_STATE_H
