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

MATCHER_P(IOStateUnorderedEq, io_state, "")
{
  return ExplainMatchResult(ContainerUnorderedEq(io_state.input()), arg.input(), result_listener) &&
         ExplainMatchResult(ContainerUnorderedEq(io_state.output()), arg.output(), result_listener);
}

MATCHER_P(PointwiseIOStateUnorderedEq, vec, "")
{
  return std::equal(vec.begin(), vec.end(), arg.begin(), arg.end(), [result_listener](const auto& a, const auto& b) {
    return ExplainMatchResult(IOStateUnorderedEq(b), a, result_listener);
  });
}

MATCHER_P(ScanDataEqual, scan, "")
{
  return arg.getScanCounter() == scan.getScanCounter() && arg.getScanResolution() == scan.getScanResolution() &&
         arg.getMinScanAngle() == scan.getMinScanAngle() && arg.getMaxScanAngle() == scan.getMaxScanAngle() &&
         Matches(PointwiseDoubleEq(scan.getMeasurements()))(arg.getMeasurements()) &&
         Matches(PointwiseDoubleEq(scan.getIntensities()))(arg.getIntensities()) &&
         ExplainMatchResult(PointwiseIOStateUnorderedEq(scan.getIOStates()), arg.getIOStates(), result_listener);
}

MATCHER_P2(TimestampInExpectedTimeframe, reference_scan, reference_timestamp, "")
{
  const int64_t elapsed_time{ util::getCurrentTime() - reference_timestamp };
  *result_listener << "where the elapsed time is " << elapsed_time << " nsec";
  return arg.getTimestamp() > reference_scan.getTimestamp() &&
         arg.getTimestamp() < (reference_scan.getTimestamp() + elapsed_time);
}

MATCHER_P(IOPinDataEq, ref_pin, "")
{
  return Matches(Pointwise(Eq(), ref_pin.input))(arg.input) && Matches(Pointwise(Eq(), ref_pin.output))(arg.output);
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
