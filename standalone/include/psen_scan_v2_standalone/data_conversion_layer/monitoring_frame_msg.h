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

#ifndef PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_H
#define PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_H

#include <cstdint>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <boost/optional.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/util/format_range.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
/**
 * @brief Namespace containing all things about the MonitoringFrame data strucure.
 *
 * The MonitoringFrame is a data structure fitting into a UDP paket, containing for example
 * distance and intensity data, and also diagnostic data. Take a look into the protocol
 * description for more info about the MonitoringFrame data structure.
 */
namespace monitoring_frame
{
static constexpr uint8_t MAX_SCANNER_ID{ configuration::VALID_SCANNER_IDS.size() - 1 };

/**
 * @brief Higher level data type representing a single monitoring frame.
 *
 * It contains all information deserialized from a single monitoring frame sent by the scanner hardware.
 *
 * @see monitoring_frame
 */
class Message
{
public:
  // always included
  configuration::ScannerId scanner_id_{ configuration::ScannerId::master };
  util::TenthOfDegree from_theta_{ 0 };
  util::TenthOfDegree resolution_{ 1 };
  std::vector<double> measurements_;

  // optional
  boost::optional<uint32_t> scan_counter_;
  boost::optional<uint8_t> active_zoneset_;
  std::vector<double> intensities_;
  std::vector<diagnostic::Message> diagnostic_messages_;
};

/**
 * @brief Wrapping class for a Message and its corresponding timestamp
 * @see Message
 **/
struct MessageStamped
{
  MessageStamped(const Message& message, const int64_t timestamp) : msg_(message), stamp_(timestamp){};
  Message msg_;
  int64_t stamp_;
};

inline std::ostream& operator<<(std::ostream& os, const Message& msg)
{
  return os << fmt::format("monitoring_frame::Message(fromTheta = {} deg, resolution = {} deg, scanCounter = "
                           "{}, active_zoneset = {}, measurements = {}, intensities = {}, diagnostics = {})",
                           msg.from_theta_.value() / 10.,
                           msg.resolution_.value() / 10.,
                           msg.scan_counter_.is_initialized() ? fmt::format("{}", *msg.scan_counter_) : "",
                           msg.active_zoneset_.is_initialized() ? fmt::format("{}", *msg.active_zoneset_) : "",
                           util::formatRange(msg.measurements_),
                           util::formatRange(msg.intensities_),
                           util::formatRange(msg.diagnostic_messages_));
}

}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_H
