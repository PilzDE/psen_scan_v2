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

#include <tinyxml2.h>

#include "psen_scan_v2_standalone/configuration/xml_configuation_parser.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"

#include <iostream>

namespace psen_scan_v2_standalone
{
namespace configuration
{
ZoneSetConfiguration XMLConfigurationParser::parse(const tinyxml2::XMLDocument& doc)
{
  ZoneSetConfiguration zoneset_config;

  tinyxml2::XMLConstHandle docHandle(&doc);
  tinyxml2::XMLConstHandle xml_set_info_handle = docHandle.FirstChildElement("MIB")
                                                     .FirstChildElement("scannerDescr")
                                                     .FirstChildElement("zoneSetDefinition")
                                                     .FirstChildElement("zoneSetInfo");

  // Loop over all <zoneSetInfo>
  const tinyxml2::XMLElement* xml_set_element = xml_set_info_handle.ToElement();

  if (!xml_set_element)
  {
    throw XMLConfigurationParserException(
        "Could not parse. Chain MIB->scannerDescr->zoneSetDefinition->zoneSetInfo not complete.");
  }

  while (xml_set_element)
  {
    // Loop over <zoneSetDetail> to generate a ZoneSet
    ZoneSet set;
    const tinyxml2::XMLElement* xml_set_detail_element = xml_set_element->FirstChildElement("zoneSetDetail");

    if (!xml_set_detail_element)
    {
      throw XMLConfigurationParserException(
          "Could not parse. Chain MIB->scannerDescr->zoneSetDefinition->zoneSetInfo->zoneSetDetail not complete.");
    }

    while (xml_set_detail_element)  // TODO do-while?
    {
      const tinyxml2::XMLElement* xml_set_detail_type_element = xml_set_detail_element->FirstChildElement("type");
      if (!xml_set_detail_type_element)
      {
        throw XMLConfigurationParserException("Could not parse. At least one <zoneSetDetail> is missing a <type>.");
      }
      const tinyxml2::XMLElement* xml_set_detail_ro_element = xml_set_detail_element->FirstChildElement("ro");
      if (!xml_set_detail_ro_element)
      {
        throw XMLConfigurationParserException("Could not parse. At least one <zoneSetDetail> is missing a <ro>.");
      }

      const char* xml_set_detail_ro_element_str = xml_set_detail_ro_element->GetText();
      if (xml_set_detail_ro_element_str == nullptr || strlen(xml_set_detail_ro_element_str) == 0)
      {
        throw XMLConfigurationParserException("Could not parse. <ro> element is empty.");
      }

      const char* xml_set_detail_type_element_str = xml_set_detail_type_element->GetText();
      if (xml_set_detail_type_element_str == nullptr || strlen(xml_set_detail_type_element_str) == 0)
      {
        throw XMLConfigurationParserException("Could not parse. <type> element is empty.");
      }

      if (strcmp(xml_set_detail_type_element_str, "roOSSD1") == 0)
      {
        set.ro_safety_ = ro_string_to_vec(xml_set_detail_ro_element_str);
      }
      else if (strcmp(xml_set_detail_type_element_str, "warn1") == 0)
      {
        set.ro_warn_ = ro_string_to_vec(xml_set_detail_ro_element_str);
      }
      else
      {
        throw XMLConfigurationParserException("Could not parse. Invalid <type> must be \"roOSSD1\" or \"warn1\".");
      }

      // Move to next <zoneSetDetail>
      xml_set_detail_element = xml_set_detail_element->NextSiblingElement("zoneSetDetail");
    }

    zoneset_config.zonesets_.push_back(set);
    xml_set_element = xml_set_element->NextSiblingElement("zoneSetInfo");
  }

  // Parse speedrange if enc is enabled
  const tinyxml2::XMLElement* enc_enabled_element = docHandle.FirstChildElement("MIB")
                                                        .FirstChildElement("clusterDescr")
                                                        .FirstChildElement("zoneSetConfiguration")
                                                        .FirstChildElement("encEnable")
                                                        .ToElement();
  if (!enc_enabled_element)
  {
    throw XMLConfigurationParserException(
        "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->encEnabled is broken.");
  }
  else
  {
    bool enc_enabled;
    if (enc_enabled_element->QueryBoolText(&enc_enabled) != tinyxml2::XML_SUCCESS)
    {
      throw XMLConfigurationParserException(
          "Could not parse. Value inside <encEnable> could not be evaluated to true or false");
    }
    if (enc_enabled)
    {
      const tinyxml2::XMLElement* xml_zone_set_select_element = docHandle.FirstChildElement("MIB")
                                                                    .FirstChildElement("clusterDescr")
                                                                    .FirstChildElement("zoneSetConfiguration")
                                                                    .FirstChildElement("zoneSetSelCode")
                                                                    .FirstChildElement("zoneSetSelector")
                                                                    .ToElement();

      if (!xml_zone_set_select_element)
      {
        throw XMLConfigurationParserException(
            "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->zoneSetSelCode->zoneSetSelector is "
            "broken.");
      }
      size_t zoneset_id_counter = 0;
      while (xml_zone_set_select_element)
      {
        const tinyxml2::XMLElement* xml_zone_set_speed_range_element =
            xml_zone_set_select_element->FirstChildElement("zoneSetSpeedRange");

        if (!xml_zone_set_speed_range_element)
        {
          throw XMLConfigurationParserException("Could not parse. Missing <zoneSetSpeedRange> below <zoneSetSelector>");
        }

        const tinyxml2::XMLElement* xml_zone_set_select_element_min =
            xml_zone_set_speed_range_element->FirstChildElement("minSpeed");
        if (!xml_zone_set_select_element_min)
        {
          throw XMLConfigurationParserException("Could not parse. Missing <minSpeed> below <zoneSetSpeedRange>");
        }

        const tinyxml2::XMLElement* xml_zone_set_select_element_max =
            xml_zone_set_speed_range_element->FirstChildElement("maxSpeed");
        if (!xml_zone_set_select_element_max)
        {
          throw XMLConfigurationParserException("Could not parse. Missing <maxSpeed> below <zoneSetSpeedRange>");
        }

        unsigned int min_speed, max_speed;
        if (xml_zone_set_select_element_min->QueryUnsignedText(&min_speed) != tinyxml2::XML_SUCCESS)
        {
          throw XMLConfigurationParserException("Could not parse. Value <minSpeed> invalid.");
        }

        if (xml_zone_set_select_element_max->QueryUnsignedText(&max_speed) != tinyxml2::XML_SUCCESS)
        {
          throw XMLConfigurationParserException("Could not parse. Value <maxSpeed> invalid.");
        }

        ZoneSetSpeedRange speed_range(min_speed, max_speed);
        try
        {
          zoneset_config.zonesets_.at(zoneset_id_counter).speed_range_ = speed_range;
        }
        catch (const std::out_of_range& e)
        {
          throw XMLConfigurationParserException("Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable> "
                                                "but there are more speedRanges than defined zones.");
        }

        xml_zone_set_select_element = xml_zone_set_select_element->NextSiblingElement("zoneSetSelector");
        zoneset_id_counter++;
      }

      // If speed_ranges are defined they should be for every zoneset
      for (const auto& zoneset : zoneset_config.zonesets_)
      {
        if (!zoneset.speed_range_)
        {
          throw XMLConfigurationParserException("Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable> "
                                                "but there are more speedRanges than defined zones.");
        }
      }
    }
  }

  return zoneset_config;
}

ZoneSetConfiguration XMLConfigurationParser::parseFile(const char* filename)
{
  tinyxml2::XMLDocument doc;
  auto parse_result = doc.LoadFile(filename);
  if (parse_result != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException("Could not parse file.");
  }

  return parse(doc);
}

ZoneSetConfiguration XMLConfigurationParser::parseString(const char* xml)
{
  tinyxml2::XMLDocument doc;
  auto parse_result = doc.Parse(xml);
  if (parse_result != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException("Could not parse content.");
  }

  return parse(doc);
}

}  // namespace configuration
}  // namespace psen_scan_v2_standalone
