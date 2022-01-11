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

#ifndef PSEN_SCAN_V2_TEST_TEST_DATA_H
#define PSEN_SCAN_V2_TEST_TEST_DATA_H

#include <cstdint>
#include <vector>

#include <boost/optional.hpp>

namespace psen_scan_v2_test
{
namespace test_data
{
class TestDatum
{
public:
  TestDatum(const uint32_t scan_counter, const int64_t timestamp, const int64_t callback_invocation_time);

  void firstFrameTime(const int64_t first_frame_time);
  bool isComplete() const;

  uint32_t scanCounter() const;
  int64_t timestamp() const;
  int64_t callbackInvocationTime() const;
  int64_t firstFrameTime() const;

private:
  uint32_t scan_counter_;
  int64_t timestamp_;
  int64_t callback_invocation_time_;
  boost::optional<int64_t> first_frame_time_;
};

using TestData = std::vector<TestDatum>;

}  // namespace test_data
}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_TEST_DATA_H
