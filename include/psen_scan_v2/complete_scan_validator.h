// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_COMPLETE_SCAN_VALIDATOR_H
#define PSEN_SCAN_V2_COMPLETE_SCAN_VALIDATOR_H

#include <boost/optional.hpp>

#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/logging.h"

namespace psen_scan_v2
{
namespace monitoring_frame
{
//! @brief Validates complete scan rounds and detects if MonitoringFrames are missing for a complete scan or
//! if to many MonitoringFrames were received.
class ScanValidator
{
public:
  enum class Result
  {
    //! Less than expected MonitoringFrames received.
    undersaturated,
    //! To much MonitoringFrames received.
    oversaturated,
    //! Complete scan is valid.
    valid
  };

  using OptionalResult = boost::optional<Result>;

public:
  /**
   * @brief Whenever a scan round is complete this function validates the finished scan and
   * returns the corresponding result.
   *
   * @note:
   * A scan round is considered to be complete whenever the next scan round starts.
   *
   * @param msg Current received MonitoringFrames.
   * @param num_expected_msgs Number of MonitoringFrames which are needed for a scan round to be complete.
   * @return See Result type.
   */
  OptionalResult validate(const monitoring_frame::Message& msg, const uint32_t& num_expected_msgs);

  /**
   * @brief Readies the validator for a new validation round. This function has to be called whenever
   * there is an expected brake in the receiving of MonitoringFrames.
   */
  void reset();

private:
  /**
   * @brief Helper class storing all information needed to evaluate a complete scan round.
   */
  class ScanRoundInfo
  {
  public:
    ScanRoundInfo(const uint32_t& scan_counter);

  public:
    Result validate(const uint32_t expected_num_msgs) const;
    uint32_t scanCounter() const;
    ScanRoundInfo& operator++();

  private:
    uint32_t scan_counter_{ 0 };
    unsigned short int num_received_msgs_{ 0 };
  };

private:
  boost::optional<ScanRoundInfo> curr_scan_round_;
};

inline ScanValidator::ScanRoundInfo::ScanRoundInfo(const uint32_t& scan_counter) : scan_counter_(scan_counter)
{
}

inline ScanValidator::Result ScanValidator::ScanRoundInfo::validate(const uint32_t expected_num_msgs) const
{
  if (num_received_msgs_ < expected_num_msgs)
  {
    return Result::undersaturated;
  }

  if (num_received_msgs_ == expected_num_msgs)
  {
    return Result::valid;
  }

  return Result::oversaturated;
}

inline uint32_t ScanValidator::ScanRoundInfo::scanCounter() const
{
  return scan_counter_;
}

inline ScanValidator::ScanRoundInfo& ScanValidator::ScanRoundInfo::operator++()
{
  ++num_received_msgs_;
  return *this;
}

inline void ScanValidator::reset()
{
  curr_scan_round_ = boost::none;
}

inline ScanValidator::OptionalResult ScanValidator::validate(const monitoring_frame::Message& msg,
                                                             const uint32_t& num_expected_msgs)
{
  if (curr_scan_round_ == boost::none)
  {
    curr_scan_round_ = ScanRoundInfo(msg.scanCounter());
  }

  auto& curr_info = curr_scan_round_.value();
  if (curr_info.scanCounter() == msg.scanCounter())
  {
    ++curr_info;
    return boost::none;
  }

  const auto old_info = curr_info;
  curr_scan_round_ = ScanRoundInfo(msg.scanCounter());
  ++(curr_scan_round_.value());
  return old_info.validate(num_expected_msgs);
}

}  // namespace monitoring_frame

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_COMPLETE_SCAN_VALIDATOR_H
