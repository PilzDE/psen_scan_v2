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
#ifndef PSEN_SCAN_V2_DIST_H
#define PSEN_SCAN_V2_DIST_H

#include <fmt/format.h>

#include <math.h>
#include <numeric>
#include <vector>

namespace psen_scan_v2_test
{
class NormalDist
{
public:
  void update(const double& value)
  {
    values_.push_back(value);
  };

  double mean() const
  {
    double sum = std::accumulate(values_.begin(), values_.end(), 0.0);
    double mean = sum / values_.size();

    return mean;
  }

  double stdev() const
  {
    double sq_sum = std::inner_product(values_.begin(), values_.end(), values_.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / values_.size() - mean() * mean());

    // This is small hack, a later comparision using the bhattacharyya_distance
    // would result in inf distance
    stdev = std::max(stdev, 0.000000001);

    return stdev;
  }

  size_t N() const
  {
    return values_.size();
  }

private:
  std::vector<double> values_;
};

double bhattacharyya_distance(const NormalDist& dist1, const NormalDist& dist2)
{
  const double sigma1_sqr = pow(dist1.stdev(), 2);
  const double sigma2_sqr = pow(dist2.stdev(), 2);

  return 0.25 * log(0.25 * ((sigma1_sqr / sigma2_sqr) + (sigma2_sqr / sigma1_sqr) + 2)) +
         0.25 * ((pow(dist1.mean() - dist2.mean(), 2) / (sigma1_sqr + sigma2_sqr)));
}

}  // namespace psen_scan_v2_test

std::ostream& operator<<(std::ostream& os, const psen_scan_v2_test::NormalDist& dist)
{
  os << fmt::format("NormalDist(mean: {:+.3f}, stdev: {:+.3f} N: {})", dist.mean(), dist.stdev(), dist.N());
  return os;
}

#endif  // PSEN_SCAN_V2_DIST_H