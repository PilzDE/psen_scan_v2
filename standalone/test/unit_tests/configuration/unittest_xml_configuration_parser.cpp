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
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"
#include "psen_scan_v2_standalone/util/expectations.h"

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

TEST_F(XmlConfiguationParserTest, settingDefaultResolution)
{
  // Unfortunatly this is only known implicitly
  configuration::XMLConfigurationParser parser;
  configuration::ZoneSetConfiguration zoneset_config =
      parser.parseFile("testfiles/unittest_xml_configuration_parser-testfile-with-speedrange.xml");

  EXPECT_EQ(zoneset_config.zonesets_[0].resolution_,
            psen_scan_v2_standalone::configuration::DEFAULT_ZONESET_ANGLE_STEP);
  EXPECT_EQ(zoneset_config.zonesets_[1].resolution_,
            psen_scan_v2_standalone::configuration::DEFAULT_ZONESET_ANGLE_STEP);
}

TEST_F(XmlConfiguationParserTest, throwOnInvalidXML)
{
  configuration::XMLConfigurationParser parser;
  EXPECT_THROW_AND_WHAT(
      parser.parseString("NON XML STRING"), configuration::XMLConfigurationParserException, "Could not parse content.");
}

TEST_F(XmlConfiguationParserTest, correctParseAllFieldTypes)
{
  configuration::XMLConfigurationParser parser;
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
  configuration::ZoneSetConfiguration zoneset_config = parser.parseString(xml.c_str());
  ASSERT_EQ(zoneset_config.zonesets_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety1_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety2_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].safety3_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].warn1_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].warn2_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].muting1_.size(), 1);
  EXPECT_EQ(zoneset_config.zonesets_[0].muting2_.size(), 1);

  EXPECT_EQ(zoneset_config.zonesets_[0].safety1_[0], 0x0001);  // Reverse defined in <ro>
  EXPECT_EQ(zoneset_config.zonesets_[0].safety2_[0], 0x0002);  //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].safety3_[0], 0x0003);  //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].warn1_[0], 0x0004);    //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].warn2_[0], 0x0005);    //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].muting1_[0], 0x0006);  //  ''
  EXPECT_EQ(zoneset_config.zonesets_[0].muting2_[0], 0x0007);  //  ''

  EXPECT_EQ(zoneset_config.zonesets_[0].speed_range_, configuration::ZoneSetSpeedRange(-5, -4));
}

TEST_F(XmlConfiguationParserTest, missingChainToZoneSetInfo)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Chain MIB->scannerDescr->zoneSetDefinition->zoneSetInfo not complete.");
}

TEST_F(XmlConfiguationParserTest, missingChainZoneSetDetail)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(
      parser.parseString(xml.c_str()),
      configuration::XMLConfigurationParserException,
      "Could not parse. Chain MIB->scannerDescr->zoneSetDefinition->zoneSetInfo->zoneSetDetail not complete.");
}

TEST_F(XmlConfiguationParserTest, missingChainZoneSetType)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. At least one <zoneSetDetail> is missing a <type>.");
}

TEST_F(XmlConfiguationParserTest, missingChainZoneSetRO)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. At least one <zoneSetDetail> is missing a <ro>.");
}
TEST_F(XmlConfiguationParserTest, emptyRO)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. <ro> element is empty.");
}

TEST_F(XmlConfiguationParserTest, wrongType)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Invalid <type> must be \"roOSSD1\" or \"warn1\".");
}

TEST_F(XmlConfiguationParserTest, emptyType)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. <type> element is empty.");
}

TEST_F(XmlConfiguationParserTest, missingEncEnable)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->encEnabled is broken.");
}

TEST_F(XmlConfiguationParserTest, emptyEncEnable)
{
  configuration::XMLConfigurationParser parser;
  const std::string xml = "<MIB>"
                          "  <clusterDescr>"
                          "    <zoneSetConfiguration>"
                          "      <encEnable></encEnable>"
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Value inside <encEnable> could not be evaluated to true or false");
}

TEST_F(XmlConfiguationParserTest, invalidEncEnableValue)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Value inside <encEnable> could not be evaluated to true or false");
}

TEST_F(XmlConfiguationParserTest, missingZoneSetSelector)
{
  configuration::XMLConfigurationParser parser;
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
      parser.parseString(xml.c_str()),
      configuration::XMLConfigurationParserException,
      "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->zoneSetSelCode->zoneSetSelector is broken.");
}

TEST_F(XmlConfiguationParserTest, missingZoneSetSpeedRange)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Missing <zoneSetSpeedRange> below <zoneSetSelector>");
}

TEST_F(XmlConfiguationParserTest, missingMinSpeed)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Missing <minSpeed> below <zoneSetSpeedRange>");
}

TEST_F(XmlConfiguationParserTest, invalidMinSpeed)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Value <minSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, invalidMinSpeedEmpty)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Value <minSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, missingMaxSpeed)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Missing <maxSpeed> below <zoneSetSpeedRange>");
}

TEST_F(XmlConfiguationParserTest, invalidMaxSpeed)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Value <maxSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, invalidMaxSpeedEmpty)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Could not parse. Value <maxSpeed> invalid.");
}

TEST_F(XmlConfiguationParserTest, invalidSpeedRange)
{
  configuration::XMLConfigurationParser parser;
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
  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::ZoneSetSpeedRangeException,
                        "Invalid speedrange min: 10 > max: 9");
}

TEST_F(XmlConfiguationParserTest, moreSpeedRangesThanZoneSets)
{
  configuration::XMLConfigurationParser parser;
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

  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable> but there are more "
                        "speedRanges than defined zones.");
}

TEST_F(XmlConfiguationParserTest, lessSpeedRangesThanZoneSets)
{
  configuration::XMLConfigurationParser parser;
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

  EXPECT_THROW_AND_WHAT(parser.parseString(xml.c_str()),
                        configuration::XMLConfigurationParserException,
                        "Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable> but there are more "
                        "speedRanges than defined zones.");
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
