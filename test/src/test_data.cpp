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

#include <cstdint>

#include <boost/optional.hpp>

#include "psen_scan_v2/test_data.h"

namespace psen_scan_v2_test
{
namespace test_data
{
TestDatum::TestDatum(const uint32_t scan_counter, const int64_t timestamp, const int64_t callback_invocation_time)
  : scan_counter_(scan_counter), timestamp_(timestamp), callback_invocation_time_(callback_invocation_time)
{
}

void TestDatum::firstFrameTime(const int64_t first_frame_time)
{
  first_frame_time_ = first_frame_time;
}

bool TestDatum::isComplete() const
{
  return first_frame_time_.is_initialized();
}

uint32_t TestDatum::scanCounter() const
{
  return scan_counter_;
}

int64_t TestDatum::timestamp() const
{
  return timestamp_;
}

int64_t TestDatum::callbackInvocationTime() const
{
  return callback_invocation_time_;
}

int64_t TestDatum::firstFrameTime() const
{
  return first_frame_time_.get();
}

}  // namespace test_data
}  // namespace psen_scan_v2_test
