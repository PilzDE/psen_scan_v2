// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_BUILDER_H
#define PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_BUILDER_H

#include <cstdint>
#include <vector>

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
class MessageBuilder
{
public:
  Message build();
  operator Message();

public:
  MessageBuilder& scannerId(configuration::ScannerId scanner_id);
  MessageBuilder& fromTheta(const util::TenthOfDegree& from_theta);
  MessageBuilder& resolution(const util::TenthOfDegree& resolution);
  MessageBuilder& measurements(const std::vector<double>& measurements);
  MessageBuilder& scanCounter(uint32_t scan_counter);
  MessageBuilder& activeZoneset(uint8_t active_zoneset);
  MessageBuilder& intensities(const std::vector<double>& intensities);
  MessageBuilder& diagnosticMessages(const std::vector<diagnostic::Message>& diagnostic_messages);
  MessageBuilder& iOPinData(const io::PinData& io_pin_data);

private:
  Message msg_;
};

inline Message MessageBuilder::build()
{
  return msg_;
}

inline MessageBuilder::operator Message()
{
  return build();
}

inline MessageBuilder& MessageBuilder::scannerId(configuration::ScannerId scanner_id)
{
  msg_.scanner_id_ = scanner_id;
  return *this;
}

inline MessageBuilder& MessageBuilder::fromTheta(const util::TenthOfDegree& from_theta)
{
  msg_.from_theta_ = from_theta;
  return *this;
}

inline MessageBuilder& MessageBuilder::resolution(const util::TenthOfDegree& resolution)
{
  msg_.resolution_ = resolution;
  return *this;
}

inline MessageBuilder& MessageBuilder::measurements(const std::vector<double>& measurements)
{
  msg_.measurements_ = measurements;
  return *this;
}

inline MessageBuilder& MessageBuilder::scanCounter(uint32_t scan_counter)
{
  msg_.scan_counter_ = scan_counter;
  return *this;
}

inline MessageBuilder& MessageBuilder::activeZoneset(uint8_t active_zoneset)
{
  msg_.active_zoneset_ = active_zoneset;
  return *this;
}

inline MessageBuilder& MessageBuilder::intensities(const std::vector<double>& intensities)
{
  msg_.intensities_ = intensities;
  return *this;
}

inline MessageBuilder& MessageBuilder::diagnosticMessages(const std::vector<diagnostic::Message>& diagnostic_messages)
{
  msg_.diagnostic_messages_ = diagnostic_messages;
  return *this;
}

inline MessageBuilder& MessageBuilder::iOPinData(const io::PinData& io_pin_data)
{
  msg_.io_pin_data_ = io_pin_data;
  return *this;
}
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_MSG_BUILDER_H