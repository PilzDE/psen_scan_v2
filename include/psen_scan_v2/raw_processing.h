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
#ifndef PSEN_SCAN_V2_RAW_PROCESSING_H
#define PSEN_SCAN_V2_RAW_PROCESSING_H

#include <sstream>
namespace psen_scan_v2
{
namespace raw_processing
{
class StringStreamFailure;

template <typename T>
inline void write(std::ostringstream& os, const T& data)
{
  os.write(reinterpret_cast<const char*>(&data), sizeof(T));
}

template <typename T>
inline void read(std::istringstream& is, T& data)
{
  is.read(reinterpret_cast<char*>(&data), sizeof(T));
  if (!is)
  {
    std::ostringstream os;
    os << "Failure reading " << sizeof(T) << " characters from input stream, could only read " << is.gcount() << ".";
    throw raw_processing::StringStreamFailure(os.str());
  }
}

template <typename RawType, typename ReturnType>
inline void read(std::istringstream& is, ReturnType& data, std::function<ReturnType(RawType)> conversion_fcn)
{
  RawType raw_data;
  read<RawType>(is, raw_data);

  data = conversion_fcn(raw_data);
}

class StringStreamFailure : public std::runtime_error
{
public:
  StringStreamFailure(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

}  // namespace raw_processing
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_RAW_PROCESSING_H
