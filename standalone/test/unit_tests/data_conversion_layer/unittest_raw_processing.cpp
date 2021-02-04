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

#include <gtest/gtest.h>
#include <gtest/gtest-typed-test.h>
#include <sstream>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;

template <typename T>
class RawProcessingTest : public ::testing::Test
{
  using TestType = T;
};

using MyTypes = testing::Types<uint8_t, uint16_t, uint32_t, uint64_t>;
#ifdef TYPED_TEST_SUITE  // in this case TYPED_TEST_CASE is deprecated
TYPED_TEST_SUITE(RawProcessingTest, MyTypes);
#else
TYPED_TEST_CASE(RawProcessingTest, MyTypes);
#endif

TYPED_TEST(RawProcessingTest, write)
{
  std::ostringstream os;
  TypeParam data{ 123 };
  data_conversion_layer::raw_processing::write(os, data);

  EXPECT_EQ(os.str().length(), sizeof(data));

  TypeParam data_returned;
  std::string data_str(os.str());
  std::copy(data_str.begin(), data_str.end(), reinterpret_cast<char*>(&data_returned));
  EXPECT_EQ(data_returned, data);
}

TYPED_TEST(RawProcessingTest, read)
{
  std::ostringstream os;
  TypeParam data{ 123 };
  os.write(reinterpret_cast<const char*>(&data), sizeof(TypeParam));
  std::istringstream is{ os.str() };

  TypeParam data_read;
  data_conversion_layer::raw_processing::read(is, data_read);

  EXPECT_EQ(data_read, data);
}

TYPED_TEST(RawProcessingTest, readTooMuch)
{
  std::ostringstream os;
  TypeParam data{ 123 };
  os.write(reinterpret_cast<const char*>(&data), sizeof(TypeParam));
  std::istringstream is{ os.str() };

  TypeParam data_read;
  data_conversion_layer::raw_processing::read(is, data_read);
  EXPECT_THROW(data_conversion_layer::raw_processing::read(is, data_read), std::exception);
}

TYPED_TEST(RawProcessingTest, readWithConversion)
{
  std::ostringstream os;
  TypeParam data{ 123 };
  os.write(reinterpret_cast<const char*>(&data), sizeof(TypeParam));
  std::istringstream is{ os.str() };

  const TypeParam data_read = data_conversion_layer::raw_processing::read<TypeParam, TypeParam>(
      is, [](TypeParam raw_data) { return raw_data * 2; });

  EXPECT_EQ(data_read, data * 2);
}

TYPED_TEST(RawProcessingTest, readArray)
{
  std::ostringstream os;
  std::vector<TypeParam> data{ (TypeParam)123, (TypeParam)345 };
  for (const auto& d : data)
  {
    os.write(reinterpret_cast<const char*>(&d), sizeof(TypeParam));
  }

  std::istringstream is{ os.str() };

  std::vector<TypeParam> data_read;
  data_conversion_layer::raw_processing::readArray<TypeParam, TypeParam>(
      is, data_read, data.size(), [](TypeParam raw_data) { return raw_data * 2; });

  EXPECT_EQ(data_read.size(), data.size());
  for (size_t i = 0; i < data_read.size(); ++i)
  {
    EXPECT_EQ(data_read.at(i), data.at(i) * 2);
  }
}

TYPED_TEST(RawProcessingTest, readArrayTooMuch)
{
  std::ostringstream os;
  std::vector<TypeParam> data{ (TypeParam)123, (TypeParam)345 };
  for (const auto& d : data)
  {
    os.write(reinterpret_cast<const char*>(&d), sizeof(TypeParam));
  }

  std::istringstream is{ os.str() };

  std::vector<TypeParam> data_read;
  EXPECT_THROW((data_conversion_layer::raw_processing::readArray<TypeParam, TypeParam>(
                   is, data_read, data.size() + 1, [](TypeParam raw_data) { return raw_data; })),
               std::exception);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
