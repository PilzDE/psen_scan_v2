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
#include <fmt/format.h>

#include "psen_scan_v2_standalone/configuration/xml_configuration_parsing.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"

namespace psen_scan_v2_standalone
{
namespace configuration
{
namespace xml_config_parsing
{
inline const tinyxml2::XMLElement* getFirstChildElement(const tinyxml2::XMLElement* parent, const char* name)
{
  const tinyxml2::XMLElement* child = parent->FirstChildElement(name);
  if (!child)
  {
    throw XMLConfigurationParserException(
        fmt::format("Could not parse. Element <{}> is missing a child <{}>.", parent->Name(), name));
  }

  return child;
}

const char* getText(const tinyxml2::XMLElement* element)
{
  const char* element_str = element->GetText();
  if (element_str == nullptr || strlen(element_str) == 0)
  {
    throw XMLConfigurationParserException(fmt::format("Could not parse. <{}> element is empty.", element->Name()));
  }

  return element_str;
}

inline bool textIsEqual(const tinyxml2::XMLElement* element, const char* str)
{
  return strcmp(getText(element), str) == 0;
}

ZoneSet parseZoneSet(const tinyxml2::XMLElement* xml_set_element)
{
  ZoneSet set;

  const tinyxml2::XMLElement* xml_set_detail_element = getFirstChildElement(xml_set_element, "zoneSetDetail");

  while (xml_set_detail_element)
  {
    const tinyxml2::XMLElement* xml_set_detail_type_element = getFirstChildElement(xml_set_detail_element, "type");
    const tinyxml2::XMLElement* xml_set_detail_ro_element = getFirstChildElement(xml_set_detail_element, "ro");

    if (textIsEqual(xml_set_detail_type_element, "roOSSD1"))
    {
      set.safety1_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else if (textIsEqual(xml_set_detail_type_element, "roOSSD2"))
    {
      set.safety2_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else if (textIsEqual(xml_set_detail_type_element, "roOSSD3"))
    {
      set.safety3_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else if (textIsEqual(xml_set_detail_type_element, "warn1"))
    {
      set.warn1_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else if (textIsEqual(xml_set_detail_type_element, "warn2"))
    {
      set.warn2_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else if (textIsEqual(xml_set_detail_type_element, "muting1"))
    {
      set.muting1_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else if (textIsEqual(xml_set_detail_type_element, "muting2"))
    {
      set.muting2_ = ro_string_to_vec(getText(xml_set_detail_ro_element));
    }
    else
    {
      throw XMLConfigurationParserException("Could not parse. Invalid <type> must be \"roOSSD1\", \"roOSSD2\", "
                                            "\"roOSSD3\", \"warn1\", \"warn2\", \"muting1\" or \"muting2\".");
    }

    // Move to next <zoneSetDetail>
    xml_set_detail_element = xml_set_detail_element->NextSiblingElement("zoneSetDetail");
  }

  // Set default resolution for now this is only known implicitly
  set.resolution_ = DEFAULT_ZONESET_ANGLE_STEP;

  return set;
}

ZoneSetSpeedRange parseZoneSetSpeedRange(const tinyxml2::XMLElement* xml_zone_set_select_element)
{
  const tinyxml2::XMLElement* xml_zone_set_speed_range_element =
      getFirstChildElement(xml_zone_set_select_element, "zoneSetSpeedRange");

  const tinyxml2::XMLElement* xml_zone_set_select_element_min =
      getFirstChildElement(xml_zone_set_speed_range_element, "minSpeed");

  const tinyxml2::XMLElement* xml_zone_set_select_element_max =
      getFirstChildElement(xml_zone_set_speed_range_element, "maxSpeed");

  unsigned int min_speed, max_speed;
  if (xml_zone_set_select_element_min->QueryUnsignedText(&min_speed) != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException("Could not parse. Value <minSpeed> invalid.");
  }

  if (xml_zone_set_select_element_max->QueryUnsignedText(&max_speed) != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException("Could not parse. Value <maxSpeed> invalid.");
  }

  return ZoneSetSpeedRange(min_speed, max_speed);
}

bool isEncoderEnabled(const tinyxml2::XMLConstHandle& doc_handle)
{
  const tinyxml2::XMLElement* enc_enabled_element = doc_handle.FirstChildElement("MIB")
                                                        .FirstChildElement("clusterDescr")
                                                        .FirstChildElement("zoneSetConfiguration")
                                                        .FirstChildElement("encEnable")
                                                        .ToElement();
  if (!enc_enabled_element)
  {
    throw XMLConfigurationParserException(
        "Could not parse. Chain MIB->clusterDescr->zoneSetConfiguration->encEnabled is broken.");
  }

  bool enc_enabled;
  if (enc_enabled_element->QueryBoolText(&enc_enabled) != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException(
        "Could not parse. Value inside <encEnable> could not be evaluated to true or false");
  }

  return enc_enabled;
}

std::vector<ZoneSet> parseZoneSets(const tinyxml2::XMLConstHandle& doc_handle)
{
  tinyxml2::XMLConstHandle xml_set_info_handle = doc_handle.FirstChildElement("MIB")
                                                     .FirstChildElement("scannerDescr")
                                                     .FirstChildElement("zoneSetDefinition")
                                                     .FirstChildElement("zoneSetInfo");

  const tinyxml2::XMLElement* xml_set_element = xml_set_info_handle.ToElement();

  if (!xml_set_element)
  {
    throw XMLConfigurationParserException(
        "Could not parse. Chain MIB->scannerDescr->zoneSetDefinition->zoneSetInfo not complete.");
  }

  std::vector<ZoneSet> zonesets;

  while (xml_set_element)
  {
    ZoneSet set = parseZoneSet(xml_set_element);

    zonesets.push_back(set);
    xml_set_element = xml_set_element->NextSiblingElement("zoneSetInfo");
  }

  return zonesets;
}

std::vector<ZoneSetSpeedRange> parseSpeedRanges(const tinyxml2::XMLConstHandle& doc_handle)
{
  const tinyxml2::XMLElement* xml_zone_set_select_element = doc_handle.FirstChildElement("MIB")
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

  std::vector<ZoneSetSpeedRange> speed_ranges;

  while (xml_zone_set_select_element)
  {
    ZoneSetSpeedRange speed_range = parseZoneSetSpeedRange(xml_zone_set_select_element);
    speed_ranges.push_back(speed_range);

    xml_zone_set_select_element = xml_zone_set_select_element->NextSiblingElement("zoneSetSelector");
  }

  return speed_ranges;
}

ZoneSetConfiguration parseTinyXML(const tinyxml2::XMLDocument& doc)
{
  tinyxml2::XMLConstHandle doc_handle(&doc);

  std::vector<ZoneSet> zonesets = parseZoneSets(doc_handle);

  if (isEncoderEnabled(doc_handle))
  {
    std::vector<ZoneSetSpeedRange> speed_ranges = parseSpeedRanges(doc_handle);

    if (zonesets.size() == speed_ranges.size())
    {
      for (size_t i = 0; i < zonesets.size(); i++)
      {
        zonesets.at(i).speed_range_ = speed_ranges.at(i);
      }
    }
    else
    {
      throw XMLConfigurationParserException(
          fmt::format("Parsing failed. SpeedRanges are enabled by <encEnable>true</Enable>"
                      "but there are {} speedRanges and {} defined zones.",
                      speed_ranges.size(),
                      zonesets.size()));
    }
  }

  ZoneSetConfiguration zoneset_config;

  zoneset_config.zonesets_ = zonesets;

  return zoneset_config;
}

ZoneSetConfiguration parseFile(const char* filename)
{
  tinyxml2::XMLDocument doc;
  auto parse_result = doc.LoadFile(filename);
  if (parse_result != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException(fmt::format("Could not parse {}.", filename));
  }

  return parseTinyXML(doc);
}

ZoneSetConfiguration parseString(const char* xml)
{
  tinyxml2::XMLDocument doc;
  auto parse_result = doc.Parse(xml);
  if (parse_result != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException("Could not parse content.");
  }

  return parseTinyXML(doc);
}

}  // namespace xml_config_parsing
}  // namespace configuration
}  // namespace psen_scan_v2_standalone
