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

#include "psen_scan_v2_standalone/configuration/xml_configuration_parsing.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"

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
using namespace psen_scan_v2_standalone::configuration::xml_config_parsing;

namespace psen_scan_v2_standalone_test
{
TEST(XMLRoStringParsing, shouldParseValidStrings)
{
  EXPECT_THAT(ro_string_to_vec(""), testing::ElementsAre());
  EXPECT_THAT(ro_string_to_vec("D307D307D307"), testing::ElementsAre(2003, 2003, 2003));
  EXPECT_THAT(ro_string_to_vec("ED03"), testing::ElementsAre(1005));
  EXPECT_THAT(ro_string_to_vec("2B018913"), testing::ElementsAre(299, 5001));
}

TEST(XMLRoStringParsing, shouldReturnEmptyOnTooShortStrings)
{
  EXPECT_THAT(ro_string_to_vec("D30"), testing::ElementsAre());
}

TEST(XMLRoStringParsing, shouldIgnoreExtraCharacters)
{
  EXPECT_THAT(ro_string_to_vec("D307D307D307AAA"), testing::ElementsAre(2003, 2003, 2003));
  //                                        ^^^ extra characters
}

TEST(XMLRoStringParsing, shouldThrowOnNonHexInput)
{
  EXPECT_THROW(ro_string_to_vec("YYYY"), XMLConfigurationParserException);
}

class XmlConfiguationParserTest : public testing::Test
{
};

TEST_F(XmlConfiguationParserTest, shouldThrowIfFileDoesNotExist)
{
  EXPECT_THROW(parseFile("non-existing-file.xml"), XMLConfigurationParserException);
}

TEST_F(XmlConfiguationParserTest, shouldNotThrowIfFileExists)
{
  EXPECT_NO_THROW(parseFile("testfiles/unittest_xml_configuration_parser-testfile-no-speedrange.xml"));
}

TEST_F(XmlConfiguationParserTest, shouldParseAValidFile)
{
  configuration::ZoneSetConfiguration zoneset_config =
      parseFile("testfiles/unittest_xml_configuration_parser-testfile-no-speedrange.xml");
  ASSERT_EQ(zoneset_config.zonesets_.size(), 2ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety1_.size(), 550ul);

  // This represents an arc
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[0].safety1_,
                                  concat({ std::vector<unsigned long>(34, 0),
                                           std::vector<unsigned long>(486, 0x00DD),
                                           std::vector<unsigned long>(30, 0) })));
  // This represents an arc
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[0].warn1_,
                                  concat({ std::vector<unsigned long>(20, 0),
                                           std::vector<unsigned long>(520, 0x00BD),
                                           std::vector<unsigned long>(10, 0) })));

  // The next zonesets are circles
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[1].safety1_, std::vector<unsigned long>(550, 0x0298)));
  EXPECT_TRUE(AllElementsAreEqual(zoneset_config.zonesets_[1].warn1_, std::vector<unsigned long>(550, 0x02F0)));

  // No speed range defined
  EXPECT_FALSE(zoneset_config.zonesets_[0].speed_range_);
  EXPECT_FALSE(zoneset_config.zonesets_[1].speed_range_);
}

TEST_F(XmlConfiguationParserTest, shouldParseAValidFileContainingSpeedRanges)
{
  configuration::ZoneSetConfiguration zoneset_config =
      parseFile("testfiles/unittest_xml_configuration_parser-testfile-with-speedrange.xml");

  EXPECT_EQ(zoneset_config.zonesets_[0].speed_range_->min_, -10);
  EXPECT_EQ(zoneset_config.zonesets_[0].speed_range_->max_, 10);
  EXPECT_EQ(zoneset_config.zonesets_[1].speed_range_->min_, 11);
  EXPECT_EQ(zoneset_config.zonesets_[1].speed_range_->max_, 50);
}

TEST_F(XmlConfiguationParserTest, shouldSetTheDefaultResolutionOfTheZoneSetConfiguration)
{
  // Unfortunatly this is only known implicitly

  configuration::ZoneSetConfiguration zoneset_config =
      parseFile("testfiles/unittest_xml_configuration_parser-testfile-with-speedrange.xml");

  EXPECT_EQ(zoneset_config.zonesets_[0].resolution_,
            psen_scan_v2_standalone::configuration::DEFAULT_ZONESET_ANGLE_STEP);
  EXPECT_EQ(zoneset_config.zonesets_[1].resolution_,
            psen_scan_v2_standalone::configuration::DEFAULT_ZONESET_ANGLE_STEP);
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnNonXMLInput)
{
  EXPECT_THROW_AND_WHAT(parseString("NON XML STRING"), XMLConfigurationParserException, "Could not parse content.");
}

TEST_F(XmlConfiguationParserTest, shouldParseAllFieldTypes)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>-4</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>0100</ro>"
                          "          <type>roOSSD1</type>"  // Type roOSSD1 -> safety1
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>0200</ro>"
                          "          <type>roOSSD2</type>"  // Type roOSSD2 -> safety2
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>0300</ro>"
                          "          <type>roOSSD3</type>"  // Type roOSSD3 -> safety3
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>0400</ro>"
                          "          <type>warn1</type>"  // Type warn1
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>0500</ro>"
                          "          <type>warn2</type>"  // Type warn2
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>0600</ro>"
                          "          <type>muting1</type>"  // Type muting1
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>0700</ro>"
                          "          <type>muting2</type>"  // Type muting2
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  configuration::ZoneSetConfiguration zoneset_config = parseString(xml.c_str());
  ASSERT_EQ(zoneset_config.zonesets_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety1_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety2_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety3_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].warn1_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].warn2_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].muting1_.size(), 1ul);
  EXPECT_EQ(zoneset_config.zonesets_[0].muting2_.size(), 1ul);

  EXPECT_EQ(zoneset_config.zonesets_[0].safety1_[0], 0x0001ul);  // Reverse defined in <ro>
  EXPECT_EQ(zoneset_config.zonesets_[0].safety2_[0], 0x0002ul);  //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].safety3_[0], 0x0003ul);  //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].warn1_[0], 0x0004ul);    //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].warn2_[0], 0x0005ul);    //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].muting1_[0], 0x0006ul);  //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].muting2_[0], 0x0007ul);  //  ''

  EXPECT_EQ(zoneset_config.zonesets_[0].speed_range_, configuration::ZoneSetSpeedRange(-5, -4));
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingChainToZoneSetInfo)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          // Missing <zoneSetInfo>
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Chain MIB->scannerDescr->zoneSetDefinition->zoneSetInfo not complete.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingChainZoneSetDetail)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          // Missing <zoneSetDetail>
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Element <zoneSetInfo> is missing a child <zoneSetDetail>.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingChainZoneSetType)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          // <type> missing here
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "           <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Element <zoneSetDetail> is missing a child <type>.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingChainZoneSetRO)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          // <ro> missing here
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Element <zoneSetDetail> is missing a child <ro>.");
}
TEST_F(XmlConfiguationParserTest, emptyRO)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"  //<-- Only one speed range
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "  <zoneSetDefinition>"
                          "    <zoneSetInfo>"
                          "      <zoneSetDetail>"
                          "        <ro>00aa</ro>"
                          "        <type>roOSSD1</type>"
                          "      </zoneSetDetail>"
                          "      <zoneSetDetail>"
                          "        <ro></ro>"  // ro here empty
                          "        <type>roOSSD1</type>"
                          "      </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), XMLConfigurationParserException, "Could not parse. <ro> element is empty.");
}

TEST_F(XmlConfiguationParserTest, wrongType)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"  //<-- Only one speed range
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>WRONGTYPE</type>"  // Here wrong type
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Invalid <type> must be \"roOSSD1\", \"roOSSD2\", "
                        "\"roOSSD3\", \"warn1\", \"warn2\", \"muting1\" or \"muting2\".");
}

TEST_F(XmlConfiguationParserTest, emptyType)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"  //<-- Only one speed range
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type></type>"  // Here empty type
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), XMLConfigurationParserException, "Could not parse. <type> element is empty.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingEncEnable)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          // No <encEnable> enabled
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->encEnabled is broken.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowIfEncEnableIsEmpty)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable></encEnable>"  // <encEnable> empty
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Value inside <encEnable> could not be evaluated to true or false");
}

TEST_F(XmlConfiguationParserTest, shouldThrowIfEncEnableIsNotBool)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>INVALIDVALUE</encEnable>"  // Invalid value
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Value inside <encEnable> could not be evaluated to true or false");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingZoneSetSelector)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          // Missing <zoneSetSelector>
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()),
      XMLConfigurationParserException,
      "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->zoneSetSelCode->zoneSetSelector is broken.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingZoneSetSpeedRange)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          // missing <zoneSetSpeedRange>
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Element <zoneSetSelector> is missing a child <zoneSetSpeedRange>.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingMinSpeed)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          // Missing <minSpeed>
                          "            <maxSpeed>10</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Element <zoneSetSpeedRange> is missing a child <minSpeed>.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnInvalidMinSpeed)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>abc</minSpeed>"  // invalid min Speed
                          "            <maxSpeed>10</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), XMLConfigurationParserException, "Could not parse. Value <minSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowIfMinSpeedIsEmpty)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed></minSpeed>"  // Empty minSpeed
                          "            <maxSpeed>10</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), XMLConfigurationParserException, "Could not parse. Value <minSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnMissingMaxSpeed)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-10</minSpeed>"
                          // Missing <maxSpeed>
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Could not parse. Element <zoneSetSpeedRange> is missing a child <maxSpeed>.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnInvalidMaxSpeed)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-10</minSpeed>"
                          "            <maxSpeed>abc</maxSpeed>"  // invalid maxSpeed
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), XMLConfigurationParserException, "Could not parse. Value <maxSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnEmptyMaxSpeed)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>-10</minSpeed>"
                          "            <maxSpeed></maxSpeed>"  // empty maxSpeed
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), XMLConfigurationParserException, "Could not parse. Value <maxSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowOnInvalidSpeedRange)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>10</minSpeed>"  // min_speed > max_speed
                          "            <maxSpeed>9</maxSpeed>"   //
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";
  EXPECT_THROW_AND_WHAT(
      parseString(xml.c_str()), configuration::ZoneSetSpeedRangeException, "Invalid speedrange min: 10 > max: 9");
}

TEST_F(XmlConfiguationParserTest, shouldThrowIfMoreSpeedRangesThanZonesets)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"  // <-- SpeedRange #1
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "        <zoneSetSelector>"  // <-- SpeedRange #2
                          "          <zoneSetSpeedRange>"
                          "            <minSpeed>6</minSpeed>"
                          "            <maxSpeed>15</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"  //<-- Only one zoneset!
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";

  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable>"
                        "but there are 2 speedRanges and 1 defined zones.");
}

TEST_F(XmlConfiguationParserTest, shouldThrowIfLessSpeedRangesThanZoneset)
{
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable>true</encEnable>"
                          "      <zoneSetSelCode>"
                          "        <zoneSetSelector>"
                          "          <zoneSetSpeedRange>"  //<-- Only one speed range
                          "            <minSpeed>-5</minSpeed>"
                          "            <maxSpeed>5</maxSpeed>"
                          "          </zoneSetSpeedRange>"
                          "        </zoneSetSelector>"
                          "      </zoneSetSelCode>"
                          "    </zoneSetConfiguration>"
                          "  </clusterDescr>"
                          "  <scannerDescr>"
                          "    <zoneSetDefinition>"
                          "      <zoneSetInfo>"  //<-- Zoneset #1
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00aa</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "      <zoneSetInfo>"  //<-- Zoneset #2
                          "        <zoneSetDetail>"
                          "          <ro>00bb</ro>"
                          "          <type>warn1</type>"
                          "        </zoneSetDetail>"
                          "        <zoneSetDetail>"
                          "          <ro>00bb</ro>"
                          "          <type>roOSSD1</type>"
                          "        </zoneSetDetail>"
                          "      </zoneSetInfo>"
                          "    </zoneSetDefinition>"
                          "  </scannerDescr>"
                          "</MIB>";

  EXPECT_THROW_AND_WHAT(parseString(xml.c_str()),
                        XMLConfigurationParserException,
                        "Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable>"
                        "but there are 1 speedRanges and 2 defined zones.");
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
