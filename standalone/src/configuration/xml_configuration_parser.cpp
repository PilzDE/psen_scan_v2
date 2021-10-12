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
ZoneSetConfiguration XMLConfigurationParser::parse(const char* filename)
{
  tinyxml2::XMLDocument doc;
  auto parse_result = doc.LoadFile(filename);
  if (parse_result != tinyxml2::XML_SUCCESS)
  {
    throw XMLConfigurationParserException("Could not parse file");
  }

  ZoneSetConfiguration zoneset_config;

  tinyxml2::XMLConstHandle docHandle(&doc);
  tinyxml2::XMLConstHandle xml_set_info_handle = docHandle.FirstChildElement("MIB")
                                                     .FirstChildElement("scannerDescr")
                                                     .FirstChildElement("zoneSetDefinition")
                                                     .FirstChildElement("zoneSetInfo");

  // Loop over all <zoneSetInfo>
  const tinyxml2::XMLElement* xml_set_element = xml_set_info_handle.ToElement();
  while (xml_set_element)
  {
    // Loop over <zoneSetDetail> to generate a ZoneSet
    ZoneSet set;
    tinyxml2::XMLConstHandle xml_set_detail_handle = xml_set_info_handle.FirstChildElement("zoneSetDetail");
    const tinyxml2::XMLElement* xml_set_detail_element = xml_set_info_handle.ToElement();

    while (xml_set_detail_element)  // TODO do-while?
    {
      const tinyxml2::XMLElement* xml_set_detail_type_element =
          xml_set_detail_handle.FirstChildElement("type").ToElement();
      const tinyxml2::XMLElement* xml_set_detail_ro_element = xml_set_detail_handle.FirstChildElement("ro").ToElement();

      if (strcmp(xml_set_detail_type_element->GetText(), "roOSSD1") == 0)
      {
        set.ro_safety_ = ro_string_to_vec(xml_set_detail_ro_element->GetText());
      }
      else if (strcmp(xml_set_detail_type_element->GetText(), "warn1") == 0)
      {
        set.ro_warn_ = ro_string_to_vec(xml_set_detail_ro_element->GetText());
      }

      // Move to next <zoneSetDetail>
      xml_set_detail_handle = xml_set_detail_handle.NextSiblingElement("zoneSetDetail");
      xml_set_detail_element = xml_set_detail_handle.ToElement();
    }

    zoneset_config.zonesets_.push_back(set);
    xml_set_info_handle = xml_set_info_handle.NextSiblingElement("zoneSetInfo");
    xml_set_element = xml_set_info_handle.ToElement();
  }

  // Parse speedrange if enc is enabled
  const tinyxml2::XMLElement* enc_enabled_element = docHandle.FirstChildElement("MIB")
                                                        .FirstChildElement("clusterDescr")
                                                        .FirstChildElement("zoneSetConfiguration")
                                                        .FirstChildElement("encEnable")
                                                        .ToElement();
  if (enc_enabled_element)
  {
    if (strcmp(enc_enabled_element->GetText(), "true") == 0)
    {
      tinyxml2::XMLConstHandle xml_zone_set_sel_code_handle = docHandle.FirstChildElement("MIB")
                                                                  .FirstChildElement("clusterDescr")
                                                                  .FirstChildElement("zoneSetConfiguration")
                                                                  .FirstChildElement("zoneSetSelCode");

      tinyxml2::XMLConstHandle xml_zone_set_select_handle =
          xml_zone_set_sel_code_handle.FirstChildElement("zoneSetSelector");
      const tinyxml2::XMLElement* xml_zone_set_select_element = xml_zone_set_select_handle.ToElement();

      size_t zoneset_id_counter = 0;
      while (xml_zone_set_select_element)
      {
        // TODO check if <zoneSetSelCode> should be used
        // const auto zoneset_id =
        //    std::stoul(xml_zone_set_select_handle.FirstChildElement("zoneSetSelCode").ToElement()->GetText()) - 1;
        const auto min_speed = std::stol(xml_zone_set_select_handle.FirstChildElement("zoneSetSpeedRange")
                                             .FirstChildElement("minSpeed")
                                             .ToElement()
                                             ->GetText());
        const auto max_speed = std::stol(xml_zone_set_select_handle.FirstChildElement("zoneSetSpeedRange")
                                             .FirstChildElement("maxSpeed")
                                             .ToElement()
                                             ->GetText());

        ZoneSetSpeedRange speed_range(min_speed, max_speed);
        zoneset_config.zonesets_.at(zoneset_id_counter).speed_range_ = speed_range;

        xml_zone_set_select_handle = xml_zone_set_select_handle.NextSiblingElement("zoneSetSelector");
        xml_zone_set_select_element = xml_zone_set_select_handle.ToElement();
        zoneset_id_counter++;
      }
    }
  }
  else
  {
    // TODO
  }

  return zoneset_config;
}

}  // namespace configuration
}  // namespace psen_scan_v2_standalone
