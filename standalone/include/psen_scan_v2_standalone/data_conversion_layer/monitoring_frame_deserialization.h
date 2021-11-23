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
#ifndef PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_DESERIALIZATION_H
#define PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_DESERIALIZATION_H

#include <istream>
#include <ostream>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/logging.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Namespace defining all information/types needed to describe a monitoring frame.
 *
 * The scanner splits up a full rotation in several parts and sends them in separate parts and sends them via UDP
 * packages.
 * Those parts are called monitoring frames.
 *
 * Every single frame **has to** contain some general information about the scan in the fixed fields
 * and **can** contain additional data like distances or intensities in additional fields.
 *
 * @see data_conversion_layer::monitoring_frame::FixedFields
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeader
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeaderID
 * @see protocol_layer::ScanValidator
 */
namespace data_conversion_layer
{
namespace monitoring_frame
{
static constexpr uint32_t DEFAULT_DEVICE_STATUS{ 0 };
static constexpr uint32_t OP_CODE_MONITORING_FRAME{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };
static constexpr uint16_t NUMBER_OF_BYTES_SCAN_COUNTER{ 4 };
static constexpr uint16_t NUMBER_OF_BYTES_ZONE_SET{ 1 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_MEASUREMENT{ 2 };
static constexpr uint16_t NO_SIGNAL_ARRIVED{ 59956 };
static constexpr uint16_t SIGNAL_TOO_LATE{ 59958 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_INTENSITY{ 2 };

/**
 * @brief The information included in every single monitoring frame.
 */
class FixedFields
{
public:
  using DeviceStatus = uint32_t;
  using OpCode = uint32_t;
  using WorkingMode = uint32_t;
  using TransactionType = uint32_t;
  using FromTheta = util::TenthOfDegree;
  using Resolution = util::TenthOfDegree;

public:
  FixedFields(DeviceStatus device_status,
              OpCode op_code,
              WorkingMode working_mode,
              TransactionType transaction_type,
              configuration::ScannerId scanner_id,
              FromTheta from_theta,
              Resolution resolution);

public:
  DeviceStatus deviceStatus() const;
  OpCode opCode() const;
  WorkingMode workingMode() const;
  TransactionType transactionType() const;
  configuration::ScannerId scannerId() const;
  FromTheta fromTheta() const;
  Resolution resolution() const;

private:
  DeviceStatus device_status_;
  OpCode op_code_;
  WorkingMode working_mode_;
  TransactionType transaction_type_;
  configuration::ScannerId scanner_id_;
  FromTheta from_theta_;
  Resolution resolution_;
};
/**
 * @brief Definition for the type and length of an additional field in a monitoring frame.
 *
 * The exact content of a monitoring frame differs depending on the configuration.
 * Every monitoring frame can contain one or all of additional fields defined in HeaderID in any order.
 * The type and corresponding length is defined in this header.
 * Based on this information the data will be deserialized.
 *
 * @see monitoring_frame
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeaderID
 */
class AdditionalFieldHeader
{
public:
  using Id = uint8_t;
  using Length = uint16_t;

public:
  AdditionalFieldHeader(Id id, Length length);

public:
  Id id() const;
  Length length() const;

  static std::string idToString(Id id);

private:
  Id id_;
  Length length_;
};

enum class AdditionalFieldHeaderID : AdditionalFieldHeader::Id
{
  scan_counter = 0x02,
  zone_set = 0x03,
  diagnostics = 0x04,
  measurements = 0x05,
  intensities = 0x06,
  end_of_frame = 0x09
};

AdditionalFieldHeader readAdditionalField(std::istream& is, const std::size_t& max_num_bytes);

monitoring_frame::Message deserialize(const data_conversion_layer::RawData& data, const std::size_t& num_bytes);
FixedFields readFixedFields(std::istream& is);
namespace diagnostic
{
std::vector<diagnostic::Message> deserializeMessages(std::istream& is);
}

/**
 * @brief Exception thrown on problems during the extraction of the measurement data.
 */
class DecodingFailure : public std::runtime_error
{
public:
  DecodingFailure(const std::string& msg = "Error while decoding laser scanner measurement data");
};

/**
 * @brief Exception thrown on problems with the additional field: scan_counter
 *
 * The length specified in the Header of the additional field "scan_counter"
 * must be exactly as defined in NUMBER_OF_BYTES_SCAN_COUNTER for it to be converted.
 *
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeader
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeaderID
 * @see data_conversion_layer::monitoring_frame::NUMBER_OF_BYTES_SCAN_COUNTER
 */
class ScanCounterUnexpectedSize : public DecodingFailure
{
public:
  ScanCounterUnexpectedSize(const std::string& msg);
};

/**
 * @brief Exception thrown on problems with the additional field: active_zoneset
 *
 * The length specified in the Header of the additional field "active_zoneset"
 * must be exactly as defined in NUMBER_OF_BYTES_ZONE_SET for it to be converted.
 *
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeader
 * @see data_conversion_layer::monitoring_frame::AdditionalFieldHeaderID
 * @see data_conversion_layer::monitoring_frame::NUMBER_OF_BYTES_ZONE_SET
 */
class ZoneSetUnexpectedSize : public DecodingFailure
{
public:
  ZoneSetUnexpectedSize(const std::string& msg);
};

inline DecodingFailure::DecodingFailure(const std::string& msg) : std::runtime_error(msg)
{
}

inline ScanCounterUnexpectedSize::ScanCounterUnexpectedSize(const std::string& msg) : DecodingFailure(msg)
{
}

inline ZoneSetUnexpectedSize::ZoneSetUnexpectedSize(const std::string& msg) : DecodingFailure(msg)
{
}

inline AdditionalFieldHeader::Id AdditionalFieldHeader::id() const
{
  return id_;
}

inline AdditionalFieldHeader::Length AdditionalFieldHeader::length() const
{
  return length_;
}

inline FixedFields::DeviceStatus FixedFields::deviceStatus() const
{
  return device_status_;
}

inline FixedFields::OpCode FixedFields::opCode() const
{
  return op_code_;
}

inline FixedFields::WorkingMode FixedFields::workingMode() const
{
  return working_mode_;
}

inline FixedFields::TransactionType FixedFields::transactionType() const
{
  return transaction_type_;
}

inline configuration::ScannerId FixedFields::scannerId() const
{
  return scanner_id_;
}

inline FixedFields::FromTheta FixedFields::fromTheta() const
{
  return from_theta_;
}

inline FixedFields::Resolution FixedFields::resolution() const
{
  return resolution_;
}

}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_MONITORING_FRAME_DESERIALIZATION_H
