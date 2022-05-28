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
#include <gz/math/Region3.hh>
#include <gz/math/Vector3.hh>

int main(int argc, char **argv)
{
  std::cout << std::boolalpha;

  const gz::math::Region3d defaultRegion;
  // A default constructed region should be empty.
  std::cout << "The " << defaultRegion << " region is empty: "
            << defaultRegion.Empty() << std::endl;

  const gz::math::Region3d openRegion =
      gz::math::Region3d::Open(-1., -1., -1., 1., 1., 1.);
  // An open region should exclude points on its boundary.
  std::cout << "The " << openRegion << " region contains the "
            << gz::math::Vector3d::UnitX << " point: "
            << openRegion.Contains(gz::math::Vector3d::UnitX)
            << std::endl;

  const gz::math::Region3d closedRegion =
      gz::math::Region3d::Closed(0., 0., 0., 1., 1., 1.);

  // A closed region should include points on its boundary.
  std::cout << "The " << closedRegion << " region contains the "
            << gz::math::Vector3d::UnitX << " point: "
            << closedRegion.Contains(gz::math::Vector3d::UnitX)
            << std::endl;

  // Closed and open regions may intersect.
  std::cout << "Regions " << closedRegion << " and " << openRegion
            << " intersect: " << closedRegion.Intersects(openRegion)
            << std::endl;

  // The unbounded region should include all non-empty regions.
  std::cout << "The " << gz::math::Region3d::Unbounded
            << " region contains all previous non-empty intervals: "
            << (gz::math::Region3d::Unbounded.Contains(openRegion) ||
                gz::math::Region3d::Unbounded.Contains(closedRegion))
            << std::endl;

}
//! [complete]
