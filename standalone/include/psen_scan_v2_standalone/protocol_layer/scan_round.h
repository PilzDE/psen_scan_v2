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

#ifndef PSEN_SCAN_V2_STANDALONE_SCAN_ROUND_H
#define PSEN_SCAN_V2_STANDALONE_SCAN_ROUND_H

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/logging.h"

namespace psen_scan_v2_standalone
{
namespace protocol_layer
{
/**
 * @brief Buffers and validates monitoring frames for a scan round.
 *
 * Discovers if there are to many monitoring frames in a scan round.
 * Informs when a scan round ended incomplete.
 * Discovers and omits old messages.
 */
class ScanRound
{
public:
  enum class Result
  {
    /* Message was not added due to being part of a previous scan round */
    msg_was_too_old,
    /* New scan round started, but the last one was not completed */
    started_new_round_early,
    /* Too many messages arrived in this scan round */
    is_oversaturated,
    /* The expected message count is reached */
    is_complete,
    /* The expected message count is not yet reached */
    is_waiting_for_more_frames
  };

public:
  ScanRound(const uint32_t& num_expected_msgs);
  /**
   * @brief Adds the message to the current scan round if it is considered valid. Returns the current status.
   *
   * @note:
   * A scan round is considered to be complete whenever the next scan round starts or the expected number of messages
   * arived.
   *
   * @see ScanRound::Result
   *
   * @param msg Current received MonitoringFrames.
   * @return Status of current scan round
   */
  ScanRound::Result addValid(const data_conversion_layer::monitoring_frame::Message& msg);

  /**
   * @brief Readies the validator for a new validation round. This function has to be called whenever
   * there is an expected brake in the receiving of MonitoringFrames.
   */
  void reset();
  std::vector<data_conversion_layer::monitoring_frame::Message> getMsgs();

private:
  ScanRound::Result validate();

private:
  std::vector<data_conversion_layer::monitoring_frame::Message> curr_scan_round_{};
  const uint32_t& num_expected_msgs_;
  bool first_scan_round_ = true;
};

inline ScanRound::ScanRound(const uint32_t& num_expected_msgs) : num_expected_msgs_(num_expected_msgs)
{
}

inline void ScanRound::reset()
{
  curr_scan_round_.clear();
}

inline std::vector<data_conversion_layer::monitoring_frame::Message> ScanRound::getMsgs()
{
  return curr_scan_round_;
}

inline ScanRound::Result ScanRound::addValid(const data_conversion_layer::monitoring_frame::Message& msg)
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
    if (old_round == Result::is_waiting_for_more_frames && !first_scan_round_)
    {
      return Result::started_new_round_early;
    }
    first_scan_round_ = false;
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

#endif  // PSEN_SCAN_V2_STANDALONE_SCAN_ROUND_H
