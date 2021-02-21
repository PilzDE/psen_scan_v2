// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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
#ifndef PSEN_SCAN_V2_STANDALONE_START_REQUEST_SERIALIZATION_H
#define PSEN_SCAN_V2_STANDALONE_START_REQUEST_SERIALIZATION_H

#include "psen_scan_v2_standalone/data_conversion_layer/start_request.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace start_request
{
RawData serialize(const data_conversion_layer::start_request::Message& start_request, const uint32_t& seq_number);
RawData serialize(const data_conversion_layer::start_request::Message& start_request);
}  // namespace start_request
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
#endif
