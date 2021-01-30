// Copyright (c) 2019-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_ISTRING_STREAM_BUILDER_H
#define PSEN_SCAN_V2_STANDALONE_TEST_ISTRING_STREAM_BUILDER_H

#include <sstream>

namespace psen_scan_v2_standalone_test
{
class IStringStreamBuilder
{
public:
  template <typename T>
  IStringStreamBuilder& add(const T& datum);

  std::istringstream get() const;

private:
  std::ostringstream os_;
};

template <typename T>
IStringStreamBuilder& IStringStreamBuilder::add(const T& datum)
{
  os_.write(reinterpret_cast<const char*>(&datum), sizeof(T));
  return *this;
}

inline std::istringstream IStringStreamBuilder::get() const
{
  return std::istringstream(os_.str());
}

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_ISTRING_STREAM_BUILDER_H
