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
#ifndef PSEN_SCAN_V2_DECODE_EXCEPTION_H
#define PSEN_SCAN_V2_DECODE_EXCEPTION_H

#include <stdexcept>
#include <string>

namespace psen_scan_v2
{
class DecodeException : public std::runtime_error
{
public:
  DecodeException(const std::string& msg = "Decoding failed");
  virtual ~DecodeException() = default;
};

class DecodeCRCMismatchException : public DecodeException
{
public:
  DecodeCRCMismatchException(const std::string& msg = "Decoding failed! CRC did not match");
  virtual ~DecodeCRCMismatchException() = default;
};

inline DecodeException::DecodeException(const std::string& msg) : std::runtime_error(msg)
{
}

inline DecodeCRCMismatchException::DecodeCRCMismatchException(const std::string& msg) : DecodeException(msg)
{
}

}  // namespace psen_scan_v2
#endif  // PSEN_SCAN_V2_NOT_IMPLEMENTED_EXCEPTION_H
