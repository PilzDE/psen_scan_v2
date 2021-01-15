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

#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_REPLY_MSG_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_REPLY_MSG_H

#include <cstdint>

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
/**
 * @brief Contains all things needed to define and implement a data_conversion_layer::scanner_reply::Message.
 */
namespace scanner_reply
{
/**
 * @brief Higher level data type representing a reply message from the scanner.
 */
class Message
{
public:
  //! @brief Defines the possible types of reply messages which can be received from the scanner.
  enum class Type : uint32_t
  {
    unknown = 0,
    start = 0x35,
    stop = 0x36,
  };

  //! @brief Defines the operation result from the scanner.
  enum class OperationResult : uint32_t
  {
    accepted = 0x00,
    refused = 0xEB,
    unknown = 0xFF
  };

  // See protocol description
  static constexpr std::size_t SIZE{ 16u };

public:
  static constexpr Type convertToReplyType(const uint32_t& value);
  static constexpr OperationResult convertToOperationResult(const uint32_t& value);

public:
  constexpr Message(const Type& type, const OperationResult& result);

  constexpr Type type() const;
  constexpr OperationResult result() const;

private:
  const Type type_;
  const OperationResult result_;
};

inline constexpr Message::Type Message::convertToReplyType(const uint32_t& value)
{
  Type retval{ static_cast<Type>(value) };
  if ((retval != Type::start) && (retval != Type::stop))
  {
    retval = Type::unknown;
  }
  return retval;
}

inline constexpr Message::OperationResult Message::convertToOperationResult(const uint32_t& value)
{
  OperationResult retval{ static_cast<OperationResult>(value) };
  if ((retval != OperationResult::accepted) && (retval != OperationResult::refused))
  {
    retval = OperationResult::unknown;
  }
  return retval;
}

inline constexpr Message::Message(const Type& type, const OperationResult& result) : type_(type), result_(result)
{
}

inline constexpr Message::Type Message::type() const
{
  return type_;
}

inline constexpr Message::OperationResult Message::result() const
{
  return result_;
}

}  // namespace scanner_reply
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_REPLY_MSG_H
