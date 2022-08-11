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

#include "psen_scan_v2_standalone/encoder_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/encoder_data.h"

namespace psen_scan_v2_standalone
{
EncoderState::EncoderState(data_conversion_layer::monitoring_frame::encoder::EncoderData encoder_data,
                           const int64_t& timestamp)
  : encoder_data_(encoder_data), timestamp_(timestamp)
{
}

double EncoderState::getEncoder_1() const
{
  return encoder_data_.encoder_1;
}

double EncoderState::getEncoder_2() const
{
  return encoder_data_.encoder_2;
}

int64_t EncoderState::timestamp() const
{
  return timestamp_;
}

std::ostream& operator<<(std::ostream& os, const EncoderState& encoder_state)
{
  return os << "EncoderState(timestamp = " << encoder_state.timestamp_ << " nsec, " << encoder_state.encoder_data_
            << ")";
}

}  // namespace psen_scan_v2_standalone
