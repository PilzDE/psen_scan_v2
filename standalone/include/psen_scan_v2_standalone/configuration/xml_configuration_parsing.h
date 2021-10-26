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

#include <string>
#include <stdexcept>

#include <tinyxml2.h>

#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"

#ifndef PSEN_SCAN_V2_XML_CONFIGURATION_PARSER_H
#define PSEN_SCAN_V2_XML_CONFIGURATION_PARSER_H

namespace psen_scan_v2_standalone
{
namespace configuration
{
/**
 * @brief Namespace for parsing xml configuration files exported from the psenScan Configurator.
 *
 */
namespace xml_config_parsing
{
class XMLConfigurationParserException : public std::runtime_error
{
public:
  XMLConfigurationParserException(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

/**
 * @brief Converts a quadrupel \<ro\> value into the respective length in mm
 *
 * The conversion of 4 characters follows the rule "abcd" -> 0xcdab
 * Examples:
 * "D307" -> 0x07D3 -> 2003mm
 * "ED03" -> 0x03ED -> 1005mm
 * "2B01" -> 0x012B -> 299mm
 * "8913" -> 0x1389 -> 5001mm
 *
 * @param ro_value string containing a quadrupel of hex values
 * @return unsigned long length in mm
 */
unsigned long ro_value_to_uint(const std::string& ro_value)
{
  auto string_cpy = ro_value;
  std::swap(string_cpy[0], string_cpy[2]);
  std::swap(string_cpy[1], string_cpy[3]);
  return std::stoul(string_cpy, nullptr, 16);
}

/**
 * @brief Convert string from a \<ro\> element to values.
 *
 * The value in a \<ro\> element is a string with length 4*N where N is the number of distance values.
 * 4 succedding values form a set that can be transformed into the lenth in mm.
 *
 * @param ro_string
 * @return std::vector<unsigned int>
 */
std::vector<unsigned long> ro_string_to_vec(const std::string& ro_string)
{
  std::vector<unsigned long> vec;

  try
  {
    for (size_t i = 0; i < ro_string.length(); i += 4)
    {
      const auto substr = ro_string.substr(i, 4);
      if (substr.length() == 4)
      {
        vec.push_back(ro_value_to_uint(substr));
      }
    }
  }
  catch (const std::exception& e)
  {
    throw XMLConfigurationParserException(e.what());
  }

  return vec;
}

ZoneSetConfiguration parseFile(const char* filename);
ZoneSetConfiguration parseString(const char* xml);

}  // namespace xml_config_parsing
}  // namespace configuration
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_XML_CONFIGURATION_PARSER_H
