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

#include <exception>

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/logging.h"

namespace psen_scan_v2_standalone
{
namespace protocol_layer
{
/**
 * @brief Exception indicating problems with the monitoring frames of a scan round.
 */
class ScanRoundError : public std::runtime_error
{
public:
  ScanRoundError(const std::string& msg) : std::runtime_error(msg){};
};

/**
 * @brief Exception thrown if the incoming frame has an outdated scan_counter.
 */
class OutdatedMessageError : public ScanRoundError
{
public:
  OutdatedMessageError(const std::string& msg = "Detected a MonitoringFrame from an earlier round. "
                                                " The scan round will ignore it.")
    : ScanRoundError(msg){};
};

/**
 * @brief Exception thrown if a new scan round started without the last one finishing.
 */
class ScanRoundEndedEarlyError : public ScanRoundError
{
public:
  ScanRoundEndedEarlyError(const std::string& msg = "Detected a MonitoringFrame from a new scan round before the old "
                                                    "one was complete."
                                                    " Dropping the incomplete round."
                                                    " (Please check the ethernet connection or contact PILZ support if "
                                                    "the error persists.)")
    : ScanRoundError(msg){};
};

/**
 * @brief Exception thrown if a scan round has to many messages.
 */
class ScanRoundOversaturatedError : public ScanRoundError
{
public:
  ScanRoundOversaturatedError(const std::string& msg = "Received too many MonitoringFrames for one scan round.")
    : ScanRoundError(msg){};
};

/**
 * @brief Buffers and validates monitoring frames for a scan round.
 *
 * Discovers if there are to many monitoring frames in a scan round.
 * Informs when a scan round ended incomplete.
 * Discovers and omits old messages.
 */
class ScanBuffer
{
public:
  ScanBuffer(const uint32_t& num_expected_msgs);
  /**
   * @brief Adds the message to the current scan round.
   *
   * @note:
   * A scan round is considered to be complete whenever the next scan round starts or the expected number of messages
   * arrived.
   *
   * @param stamped_msg Current received MonitoringFrame.
   *
   * @throws data_conversion_layer::monitoring_frame::AdditionalFieldMissing if scan_counter is not set in
   * stamped_msg.msg_.
   */
  void add(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg);

  /**
   * @brief Readies the validator for a new validation round. This function has to be called whenever
   * there is an expected brake in the receiving of MonitoringFrames.
   */
  void reset();

  /*! deprecated: use std::vector<data_conversion_layer::monitoring_frame::MessageStamped> current_round() instead */
  [[deprecated("use std::vector<data_conversion_layer::monitoring_frame::MessageStamped> current_round() "
               "instead")]] std::vector<data_conversion_layer::monitoring_frame::MessageStamped>
  getMsgs();

  std::vector<data_conversion_layer::monitoring_frame::MessageStamped> current_round();

  bool isRoundComplete();

private:
  void startNewRound(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg);

private:
  std::vector<data_conversion_layer::monitoring_frame::MessageStamped> current_round_{};
  const uint32_t& num_expected_msgs_;
  bool first_scan_round_ = true;
};

inline ScanBuffer::ScanBuffer(const uint32_t& num_expected_msgs) : num_expected_msgs_(num_expected_msgs)
{
}

inline void ScanBuffer::reset()
{
  current_round_.clear();
}

inline std::vector<data_conversion_layer::monitoring_frame::MessageStamped> ScanBuffer::current_round()
{
  return current_round_;
}

inline std::vector<data_conversion_layer::monitoring_frame::MessageStamped> ScanBuffer::getMsgs()
{
  return current_round_;
}

inline bool ScanBuffer::isRoundComplete()
{
  return current_round_.size() == num_expected_msgs_;
}

inline void ScanBuffer::add(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg)
{
  if (current_round_.empty() || stamped_msg.msg_.scanCounter() == current_round_[0].msg_.scanCounter())
  {
    current_round_.push_back(stamped_msg);
    if (current_round_.size() > num_expected_msgs_)
    {
      throw ScanRoundOversaturatedError();
    }
  }
  else if (stamped_msg.msg_.scanCounter() > current_round_[0].msg_.scanCounter())
  {
    startNewRound(stamped_msg);
  }
  else
  {
    throw OutdatedMessageError();
  }
}

inline void ScanBuffer::startNewRound(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg)
{
  bool old_round_undersaturated = current_round_.size() < num_expected_msgs_;
  reset();
  current_round_.push_back(stamped_msg);
  if (old_round_undersaturated && !first_scan_round_)
  {
    throw ScanRoundEndedEarlyError();
  }
  first_scan_round_ = false;
}
}  // namespace protocol_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCAN_ROUND_H
