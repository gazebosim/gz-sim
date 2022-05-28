/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
//! [complete]
#include <iostream>
#include <gz/math/Interval.hh>

int main(int argc, char **argv)
{
  std::cout << std::boolalpha;

  const gz::math::Intervald defaultInterval;
  // A default constructed interval should be empty.
  std::cout << "The " << defaultInterval << " interval is empty: "
            << defaultInterval.Empty() << std::endl;

  const gz::math::Intervald openInterval =
      gz::math::Intervald::Open(-1., 1.);
  // An open interval should exclude its endpoints.
  std::cout << "The " << openInterval << " interval contains its endpoints: "
            << (openInterval.Contains(openInterval.LeftValue()) ||
                openInterval.Contains(openInterval.RightValue()))
            << std::endl;

  const gz::math::Intervald closedInterval =
      gz::math::Intervald::Closed(0., 1.);

  // A closed interval should include its endpoints.
  std::cout << "The " << closedInterval << " interval contains its endpoints: "
            << (closedInterval.Contains(closedInterval.LeftValue()) ||
                closedInterval.Contains(closedInterval.RightValue()))
            << std::endl;

  // Closed and open intervals may intersect.
  std::cout << "Intervals " << closedInterval << " and " << openInterval
            << " intersect: " << closedInterval.Intersects(openInterval)
            << std::endl;

  // The unbounded interval should include all non-empty intervals.
  std::cout << "The " << gz::math::Intervald::Unbounded
            << " interval contains all previous non-empty intervals: "
            << (gz::math::Intervald::Unbounded.Contains(openInterval) ||
                gz::math::Intervald::Unbounded.Contains(closedInterval))
            << std::endl;

}
//! [complete]
