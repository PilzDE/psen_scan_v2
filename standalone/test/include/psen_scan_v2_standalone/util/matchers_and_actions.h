// Copyright (c) 2021-2022 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_MATCHERS_AND_ACTIONS_H
#define PSEN_SCAN_V2_STANDALONE_TEST_MATCHERS_AND_ACTIONS_H

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/util/format_range.h"
#include "psen_scan_v2_standalone/util/timestamp.h"

namespace psen_scan_v2_standalone_test
{
using namespace ::testing;
using namespace ::psen_scan_v2_standalone;

ACTION_P(OpenBarrier, barrier)
{
  barrier->release();
}

ACTION_P2(OpenBarrierCond, barrier, predicate)
{
  if (predicate())
  {
    barrier->release();
  }
}

MATCHER_P(PointwiseDoubleEq, vec, "")
{
  return std::equal(vec.begin(), vec.end(), arg.begin(), arg.end(), [](const double& a, const double& b) {
    return Matches(DoubleEq(b))(a);
  });
}

MATCHER_P2(UnsafePointwiseDoubleItEq, begin, end, "")
{
  return std::equal(begin, end, arg, [](const double& a, const double& b) { return Matches(DoubleEq(b))(a); });
}

// The following matchers requires that the elements of the vectors support the ostream operator <<.
MATCHER_P(ContainerUnorderedEq, vec, "")
{
  return ExplainMatchResult(SizeIs(vec.size()), arg, result_listener) &&
         std::all_of(arg.begin(), arg.end(), [&](const auto& elem) {
           if (std::find(vec.begin(), vec.end(), elem) == vec.end())
           {
             *result_listener << ", but the expected element: " << elem << " was not found in "
                              << util::formatRange(vec);
             return false;
           }
           else
           {
             return true;
           }
         });
}

MATCHER_P(IOStateEq, io_state, "")
{
  return io_state.input() == arg.input() && io_state.output() == arg.output() &&
         io_state.timestamp() == arg.timestamp();
}

MATCHER_P(PointwiseIOStateEq, vec, "")
{
  return std::equal(vec.begin(), vec.end(), arg.begin(), arg.end(), [result_listener](const auto& a, const auto& b) {
    return ExplainMatchResult(Eq(b), a, result_listener);
  });
}

MATCHER_P(ScanDataEqual, scan, "")
{
  return arg.scanCounter() == scan.scanCounter() && arg.scanResolution() == scan.scanResolution() &&
         arg.minScanAngle() == scan.minScanAngle() && arg.maxScanAngle() == scan.maxScanAngle() &&
         Matches(PointwiseDoubleEq(scan.measurements()))(arg.measurements()) &&
         Matches(PointwiseDoubleEq(scan.intensities()))(arg.intensities()) &&
         ExplainMatchResult(PointwiseIOStateEq(scan.ioStates()), arg.ioStates(), result_listener);
}

MATCHER_P2(IOTimestampsInExpectedTimeframe, reference_ios, reference_timestamp, "")
{
  const int64_t elapsed_time{ util::getCurrentTime() - reference_timestamp };
  return std::equal(reference_ios.begin(),
                    reference_ios.end(),
                    arg.begin(),
                    arg.end(),
                    [elapsed_time](const auto& ref, const auto& act) {
                      return act.timestamp() > ref.timestamp() && act.timestamp() < (ref.timestamp() + elapsed_time);
                    });
  ;
}

MATCHER_P2(ScanTimestampsInExpectedTimeframe, reference_scan, reference_timestamp, "")
{
  const int64_t elapsed_time{ util::getCurrentTime() - reference_timestamp };
  *result_listener << "where the elapsed time is " << elapsed_time << " nsec";
  return arg.timestamp() > reference_scan.timestamp() &&
         arg.timestamp() < (reference_scan.timestamp() + elapsed_time) &&
         Matches(IOTimestampsInExpectedTimeframe(reference_scan.ioStates(), reference_timestamp))(arg.ioStates());
}

MATCHER_P(IOPinDataEq, ref_pin, "")
{
  return Matches(Pointwise(Eq(), ref_pin.input_state))(arg.input_state) &&
         Matches(Pointwise(Eq(), ref_pin.output_state))(arg.output_state);
}

MATCHER_P(MonitoringFrameEq, reference_msg, "")
{
  return arg.fromTheta() == reference_msg.fromTheta() && arg.resolution() == reference_msg.resolution() &&
         arg.scanCounter() == reference_msg.scanCounter() && arg.activeZoneset() == reference_msg.activeZoneset() &&
         Matches(PointwiseDoubleEq(reference_msg.measurements()))(arg.measurements()) &&
         Matches(PointwiseDoubleEq(reference_msg.intensities()))(arg.intensities()) &&
         Matches(Pointwise(Eq(), reference_msg.diagnosticMessages()))(arg.diagnosticMessages()) &&
         Matches(IOPinDataEq(reference_msg.iOPinData()))(arg.iOPinData());
}
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_MATCHERS_AND_ACTIONS_H
