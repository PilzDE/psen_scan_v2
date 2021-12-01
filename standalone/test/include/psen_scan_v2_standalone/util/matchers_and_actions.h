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

MATCHER_P(ScanDataEqual, scan, "")
{
  return arg.getScanCounter() == scan.getScanCounter() && arg.getScanResolution() == scan.getScanResolution() &&
         arg.getMinScanAngle() == scan.getMinScanAngle() && arg.getMaxScanAngle() == scan.getMaxScanAngle() &&
         Matches(PointwiseDoubleEq(scan.getMeasurements()))(arg.getMeasurements()) &&
         Matches(PointwiseDoubleEq(scan.getIntensities()))(arg.getIntensities());
}

MATCHER_P2(TimestampInExpectedTimeframe, reference_scan, reference_timestamp, "")
{
  const int64_t elapsed_time{ util::getCurrentTime() - reference_timestamp };
  *result_listener << "where the elapsed time is " << elapsed_time << " nsec";
  return arg.getTimestamp() > reference_scan.getTimestamp() &&
         arg.getTimestamp() < (reference_scan.getTimestamp() + elapsed_time);
}

MATCHER_P(MonitoringFrameEq, reference_msg, "")
{
  return arg.fromTheta() == reference_msg.fromTheta() && arg.resolution() == reference_msg.resolution() &&
         arg.scanCounter() == reference_msg.scanCounter() && arg.activeZoneset() == reference_msg.activeZoneset() &&
         Matches(PointwiseDoubleEq(reference_msg.measurements()))(arg.measurements()) &&
         Matches(PointwiseDoubleEq(reference_msg.intensities()))(arg.intensities()) &&
         Matches(Pointwise(Eq(), reference_msg.diagnosticMessages()))(arg.diagnosticMessages());
}
}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_MATCHERS_AND_ACTIONS_H
