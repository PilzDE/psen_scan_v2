// Copyright (c) 2022 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_ENCODER_STATE_H
#define PSEN_SCAN_V2_STANDALONE_ENCODER_STATE_H

#include <ostream>

#include "psen_scan_v2_standalone/data_conversion_layer/encoder_data.h"

namespace psen_scan_v2_standalone
{
//! @brief Represents the set of encoder reading states.
class EncoderState
{
public:
  EncoderState() = default;
  EncoderState(data_conversion_layer::monitoring_frame::encoder::EncoderData encoder_data, const int64_t& timestamp);
  //! @return double containing the reading of the encoder 1.
  double getEncoder_1() const;
  //! @return double containing the reading of the encoder 2.
  double getEncoder_2() const;
  //! @return time[ns] of the monitoring frame this state is linked to.
  int64_t timestamp() const;

private:
  data_conversion_layer::monitoring_frame::encoder::EncoderData encoder_data_{};
  int64_t timestamp_{};

public:
  friend std::ostream& operator<<(std::ostream& os, const EncoderState& encoder_state);
};

std::ostream& operator<<(std::ostream& os, const EncoderState& encoder_state);
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_ENCODER_STATE_H
