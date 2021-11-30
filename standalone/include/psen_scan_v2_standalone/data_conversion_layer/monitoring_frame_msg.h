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
#include <functional>
#include <map>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <bitset>
#include <boost/optional.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"
#include "psen_scan_v2_standalone/configuration/scanner_ids.h"

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
 * @brief Base class for exceptions thrown if an additional field was missing during deserialization of a Message.
 */
class AdditionalFieldMissing : public std::runtime_error
{
public:
  AdditionalFieldMissing(const std::string& field_name);
};

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
  configuration::ScannerId scannerId() const;
  util::TenthOfDegree fromTheta() const;
  util::TenthOfDegree resolution() const;
  //! @throw AdditionalFieldMissing if scan_counter was missing during deserialization of a Message.
  uint32_t scanCounter() const;
  //! @throw AdditionalFieldMissing if active_zoneset was missing during deserialization of a Message.
  uint8_t activeZoneset() const;
  //! @throw AdditionalFieldMissing if measurements were missing during deserialization of a Message.
  const std::vector<double>& measurements() const;
  //! @throw AdditionalFieldMissing if intensities were missing during deserialization of a Message.
  const std::vector<double>& intensities() const;
  //! @throw AdditionalFieldMissing if diagnostic_messages were missing during deserialization of a Message.
  std::vector<diagnostic::Message> diagnosticMessages() const;

  bool hasScanCounterField() const;
  bool hasActiveZonesetField() const;
  bool hasMeasurementsField() const;
  bool hasIntensitiesField() const;
  bool hasDiagnosticMessagesField() const;

private:
  // fixed fields
  configuration::ScannerId scanner_id_{ configuration::ScannerId::master };
  util::TenthOfDegree from_theta_{ 0 };
  util::TenthOfDegree resolution_{ 1 };
  // additional fields
  boost::optional<uint32_t> scan_counter_;
  boost::optional<uint8_t> active_zoneset_;
  boost::optional<std::vector<double>> measurements_;
  boost::optional<std::vector<double>> intensities_;
  boost::optional<std::vector<diagnostic::Message>> diagnostic_messages_;

public:
  friend class MessageBuilder;
  friend std::ostream& operator<<(std::ostream& os, const Message& msg);
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

inline AdditionalFieldMissing::AdditionalFieldMissing(const std::string& field_name)
  : std::runtime_error(field_name + " not set! (Contact PILZ support if the error persists.)")
{
}

std::ostream& operator<<(std::ostream& os, const Message& msg);

}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_H
