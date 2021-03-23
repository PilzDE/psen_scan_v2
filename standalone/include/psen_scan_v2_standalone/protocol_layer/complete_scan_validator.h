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

#ifndef PSEN_SCAN_V2_STANDALONE_COMPLETE_SCAN_VALIDATOR_H
#define PSEN_SCAN_V2_STANDALONE_COMPLETE_SCAN_VALIDATOR_H

#include <boost/optional.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/logging.h"

namespace psen_scan_v2_standalone
{
namespace protocol_layer
{
//! @brief Validates complete scan rounds and detects if MonitoringFrames are missing for a complete scan or
//! if to many MonitoringFrames were received.
class ScanRound
{
public:
  enum Result
  {
    msg_was_to_old,
    ended_undersaturated,
    is_oversaturated,
    is_complete,
    is_waiting_for_more_frames
  };

public:
  ScanRound(const uint32_t& num_expected_msgs);
  /**
   * @brief Adds the message to the current ScanRound if it is considered valid. Returns the status of the scan_round.
   *
   * @note:
   * A scan round is considered to be complete whenever the next scan round starts or the expected number of messages
   * arived.
   *
   * @see ScanRound::Result
   *
   * @param msg Current received MonitoringFrames.
   * @return See Result type.
   */
  ScanRound::Result add_valid(const data_conversion_layer::monitoring_frame::Message& msg);

  /**
   * @brief Readies the validator for a new validation round. This function has to be called whenever
   * there is an expected brake in the receiving of MonitoringFrames.
   */
  void reset();
  std::vector<data_conversion_layer::monitoring_frame::Message> get_msgs();

private:
  ScanRound::Result validate();

private:
  std::vector<data_conversion_layer::monitoring_frame::Message> curr_scan_round_{};
  const uint32_t& num_expected_msgs_;
};

inline ScanRound::ScanRound(const uint32_t& num_expected_msgs) : num_expected_msgs_(num_expected_msgs)
{
}

inline void ScanRound::reset()
{
  curr_scan_round_.clear();
}

inline std::vector<data_conversion_layer::monitoring_frame::Message> ScanRound::get_msgs()
{
  return curr_scan_round_;
}

inline ScanRound::Result ScanRound::add_valid(const data_conversion_layer::monitoring_frame::Message& msg)
{
  if (curr_scan_round_.empty() || msg.scanCounter() == curr_scan_round_[0].scanCounter())
  {
    curr_scan_round_.push_back(msg);
    return validate();
  }
  else if (msg.scanCounter() < curr_scan_round_[0].scanCounter())
  {
    return Result::msg_was_to_old;
  }
  else
  {
    Result old_round = validate();
    reset();
    curr_scan_round_.push_back(msg);
    if (old_round == Result::is_waiting_for_more_frames)
    {
      return Result::ended_undersaturated;
    }
    return Result::is_waiting_for_more_frames;
  }
}

inline ScanRound::Result ScanRound::validate()
{
  if (curr_scan_round_.size() == num_expected_msgs_)
  {
    return Result::is_complete;
  }
  else if (curr_scan_round_.size() < num_expected_msgs_)
  {
    return Result::is_waiting_for_more_frames;
  }
  else
  {
    return Result::is_oversaturated;
  }
}
}  // namespace protocol_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_COMPLETE_SCAN_VALIDATOR_H
