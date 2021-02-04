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
#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_REPLY_SERIALIZATION_DESERIALIZATION_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_REPLY_SERIALIZATION_DESERIALIZATION_H

#include <cstdint>
#include <stdexcept>
#include <string>
#include <sstream>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace scanner_reply
{
/**
 * @brief Exception thrown if an incorrect CRC is detected during deserialization of a
 * data_conversion_layer::scanner_reply::Message.
 */
class CRCMismatch : public std::runtime_error
{
public:
  CRCMismatch(const std::string& msg = "CRC did not match!");
};

Message deserialize(const data_conversion_layer::RawData& data);

RawData serialize(const Message& reply);
RawData serialize(const uint32_t op_code, const uint32_t res_code);

}  // namespace scanner_reply
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_REPLY_SERIALIZATION_DESERIALIZATION_H
