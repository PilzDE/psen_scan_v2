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
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/configuration/xml_configuation_parser.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"

template <typename T>
T concat(const std::initializer_list<T>& lst)
{
  T res;
  for (const auto& elem : lst)
  {
    res.insert(std::end(res), std::begin(elem), std::end(elem));
  }

  return res;
}

testing::AssertionResult AllElementsAreEqual(const std::vector<unsigned long>& val,
                                             const std::vector<unsigned long>& expectation)
{
  if (val.size() != expectation.size())
  {
    return testing::AssertionFailure() << "Size is not equal. Expected: " << expectation.size()
                                       << " actual: " << val.size();
  }

  for (size_t i = 0; i < val.size(); i++)
  {
    if (val[i] != expectation[i])
    {
      return testing::AssertionFailure() << "Value at index " << i << " not equal. Expected: " << expectation[i]
                                         << " actual: " << val[i];
    }
  }

  return testing::AssertionSuccess();
}

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
TEST(XMLRoStringParsing, parseROStringProperParsing)
{
  EXPECT_THAT(configuration::ro_string_to_vec(""), testing::ElementsAre());
  EXPECT_THAT(configuration::ro_string_to_vec("D307D307D307"), testing::ElementsAre(2003, 2003, 2003));
  EXPECT_THAT(configuration::ro_string_to_vec("ED03"), testing::ElementsAre(1005));
  EXPECT_THAT(configuration::ro_string_to_vec("2B018913"), testing::ElementsAre(299, 5001));
}

TEST(XMLRoStringParsing, stringToShort)
{
  EXPECT_THAT(configuration::ro_string_to_vec("D30"), testing::ElementsAre());
}

TEST(XMLRoStringParsing, extraCharacters)
{
  EXPECT_THAT(configuration::ro_string_to_vec("D307D307D307AAA"), testing::ElementsAre(2003, 2003, 2003));
}

TEST(XMLRoStringParsing, NONHEX)
{
  EXPECT_THROW(configuration::ro_string_to_vec("YYYY"), configuration::XMLConfigurationParserException);
}

class XmlConfiguationParserTest : public testing::Test
{
};

TEST_F(XmlConfiguationParserTest, throwIfFileDoesNotExist)
{
  configuration::XMLConfigurationParser parser;
  EXPECT_THROW(parser.parseFile("non-existing-file.xml"), configuration::XMLConfigurationParserException);
}

TEST_F(XmlConfiguationParserTest, dontThrowIfFileExists)
{
  configuration::XMLConfigurationParser parser;
  EXPECT_NO_THROW(parser.parseFile("testfiles/unittest_xml_configuration_parser-testfile-no-speedrange.xml"));
}

TEST_F(XmlConfiguationParserTest, zonesParseCorrect)
{
  configuration::XMLConfigurationParser parser;
  configuration::ZoneSetConfiguration zoneset_config =
      parser.parseFile("testfiles/unittest_xml_configuration_parser-testfile-no-speedrange.xml");
  ASSERT_EQ(zoneset_config.zonesets_.size(), 2);
  EXPECT_EQ(zoneset_config.zonesets_[0].ro_safety_.size(), 550);

  // This represents an arc
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[0].ro_safety_,
                                  concat({ std::vector<unsigned long>(34, 0),
                                           std::vector<unsigned long>(486, 0x00DD),
                                           std::vector<unsigned long>(30, 0) })));
  // This represents an arc
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[0].ro_warn_,
                                  concat({ std::vector<unsigned long>(20, 0),
                                           std::vector<unsigned long>(520, 0x00BD),
                                           std::vector<unsigned long>(10, 0) })));

  // The next zonesets are circles
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[1].ro_safety_, std::vector<unsigned long>(550, 0x0298)));
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[1].ro_warn_, std::vector<unsigned long>(550, 0x02F0)));

  // No speed range defined
  EXPECT_FALSE(zoneset_config.zonesets_[0].speed_range_);
  EXPECT_FALSE(zoneset_config.zonesets_[1].speed_range_);
}

TEST_F(XmlConfiguationParserTest, correctParsingOfSpeedRange)
{
  configuration::XMLConfigurationParser parser;
  configuration::ZoneSetConfiguration zoneset_config =
      parser.parseFile("testfiles/unittest_xml_configuration_parser-testfile-with-speedrange.xml");

  EXPECT_EQ(zoneset_config.zonesets_[0].speed_range_->min_, -10);
  EXPECT_EQ(zoneset_config.zonesets_[0].speed_range_->max_, 10);
  EXPECT_EQ(zoneset_config.zonesets_[1].speed_range_->min_, 11);
  EXPECT_EQ(zoneset_config.zonesets_[1].speed_range_->max_, 50);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
