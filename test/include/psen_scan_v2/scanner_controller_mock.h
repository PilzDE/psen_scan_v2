// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_TEST_SCANNER_CONTROLLER_MOCK_H
#define PSEN_SCAN_V2_TEST_SCANNER_CONTROLLER_MOCK_H

#include <gmock/gmock.h>

#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"

namespace psen_scan_v2_test
{
class ScannerControllerMock
{
public:
  ScannerControllerMock(const psen_scan_v2::ScannerConfiguration& scanner_config){};
  MOCK_METHOD0(start, void());
  MOCK_METHOD0(stop, void());
  MOCK_METHOD1(handleError, void(const std::string& error_msg));
  MOCK_METHOD0(sendStartRequest, void());
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_SCANNER_CONTROLLER_MOCK_H
