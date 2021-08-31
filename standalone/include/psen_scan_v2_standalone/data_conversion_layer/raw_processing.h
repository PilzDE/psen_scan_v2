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
#ifndef PSEN_SCAN_V2_STANDALONE_RAW_PROCESSING_H
#define PSEN_SCAN_V2_STANDALONE_RAW_PROCESSING_H

#include <algorithm>
#include <cmath>
#include <functional>
#include <istream>
#include <sstream>
#include <vector>

#include <fmt/format.h>

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
/**
 * @brief Contains functions, etc. needed to transform
 * higher level data to raw data (sent to or received from scanner) or vice versa.
 */
namespace raw_processing
{
/**
 * @brief Exception thrown if the incoming data from the scanner cannot be processed.
 */
class StringStreamFailure : public std::runtime_error
{
public:
  StringStreamFailure(const std::string& msg = "Error in raw data processing");
};

template <typename T>
inline void write(std::ostringstream& os, const T& data)
{
  os.write(reinterpret_cast<const char*>(&data), sizeof(T));
}

template <typename T>
inline void read(std::istream& is, T& data)
{
  is.read(reinterpret_cast<char*>(&data), sizeof(T));
  if (!is)
  {
    throw raw_processing::StringStreamFailure(
        fmt::format("Failure reading {} characters from input stream, could only read {}.", sizeof(T), is.gcount()));
  }
}

template <typename T>
inline T read(std::istream& is)
{
  T retval;
  raw_processing::read<T>(is, retval);
  return retval;
}

template <class ReturnType, class RawType>
using ConversionFunc = std::function<ReturnType(const RawType&)>;

template <typename RawType, typename ReturnType>
inline ReturnType read(std::istream& is, const ConversionFunc<ReturnType, RawType>& conversion_fcn)
{
  return conversion_fcn(raw_processing::read<RawType>(is));
}

template <typename RawType, typename ReturnType>
inline ReturnType read(std::istream& is)
{
  return ReturnType(raw_processing::read<RawType>(is));
}

template <typename RawType, typename ReturnType>
inline void readArray(std::istream& is,
                      std::vector<ReturnType>& data,
                      const size_t& number_of_samples,
                      const ConversionFunc<ReturnType, RawType>& conversion_fcn)
{
  data.reserve(number_of_samples);

  std::generate_n(std::back_inserter(data), number_of_samples, [&is, &conversion_fcn]() {
    return raw_processing::read<RawType, ReturnType>(is, conversion_fcn);
  });
}

template <typename RawType, typename ArrayElemType>
inline void writeArray(std::ostringstream& os,
                       const std::vector<ArrayElemType>& array,
                       ConversionFunc<RawType, ArrayElemType> conversion_fcn)
{
  for (const auto& elem : array)
  {
    raw_processing::write(os, conversion_fcn(elem));
  }
}

template <typename T>
inline T toArray(std::ostringstream& os)
{
  const std::string data_str(os.str());

  T raw_data;
  raw_data.reserve(data_str.length());

  std::copy(data_str.begin(), data_str.end(), std::back_inserter(raw_data));
  return raw_data;
}

inline StringStreamFailure::StringStreamFailure(const std::string& msg) : std::runtime_error(msg)
{
}

}  // namespace raw_processing
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_RAW_PROCESSING_H
