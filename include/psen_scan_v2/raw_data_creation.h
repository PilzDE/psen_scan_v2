#ifndef PSEN_SCAN_RAW_DATA_CREATION_H
#define PSEN_SCAN_RAW_DATA_CREATION_H

#include <iostream>
#include <string>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/raw_processing.h"

namespace psen_scan_v2
{
static uint32_t calcCrc(std::ostringstream& os)
{
  const DynamicSizeRawData raw_data{ raw_processing::toArray<DynamicSizeRawData>(os) };
  boost::crc_32_type crc;
  crc.process_bytes(&raw_data.at(0), raw_data.size());
  return crc.checksum();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_RAW_DATA_CREATION_H
