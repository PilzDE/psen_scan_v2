#ifndef PSEN_SCAN_RAW_DATA_CREATION_H
#define PSEN_SCAN_RAW_DATA_CREATION_H

#include <iostream>
#include <string>

#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
static DynamicSizeRawData toRawData(std::ostringstream& os)
{
  const std::string data_str(os.str());

  DynamicSizeRawData raw_data;
  raw_data.reserve(data_str.length());

  std::copy(data_str.begin(), data_str.end(), std::back_inserter(raw_data));
  return raw_data;
}

static uint32_t calcCrc(std::ostringstream& os)
{
  const DynamicSizeRawData raw_data{ psen_scan_v2::toRawData(os) };
  boost::crc_32_type crc;
  crc.process_bytes(&raw_data.at(0), raw_data.size());
  return crc.checksum();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_RAW_DATA_CREATION_H
