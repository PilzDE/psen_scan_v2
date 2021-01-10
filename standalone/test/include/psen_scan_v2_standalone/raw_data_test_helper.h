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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_RAW_DATA_TEST_HELPER_H
#define PSEN_SCAN_V2_STANDALONE_TEST_RAW_DATA_TEST_HELPER_H

#include <boost/endian/conversion.hpp>
#include <gtest/gtest.h>

namespace psen_scan_v2_standalone_test
{
enum class Endian
{
  little,
  big
};

template <typename ValueType, typename RawType>
::testing::AssertionResult
DecodingEquals(RawType const& data, std::size_t offset, ValueType expected, const Endian& endian = Endian::little)
{
  ValueType actual_val;
  memcpy(&actual_val, data.data() + offset, sizeof(ValueType));

  if (endian == Endian::big)
  {
    boost::endian::native_to_big_inplace(actual_val);
  }

  if (actual_val == expected)
  {
    return ::testing::AssertionSuccess();
  }
  else
  {
    return ::testing::AssertionFailure() << actual_val << " not equal to " << expected;
  }
}

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_RAW_DATA_TEST_HELPER_H
