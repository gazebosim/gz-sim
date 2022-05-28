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
#include <gz/math/Polynomial3.hh>
#include <gz/math/Vector4.hh>

int main(int argc, char **argv)
{
  // A default constructed polynomial should have zero coefficients.
  std::cout << "A default constructed polynomial is always: "
            << gz::math::Polynomial3d() << std::endl;

  // A constant polynomial only has an independent term.
  std::cout << "A constant polynomial only has an independent term: "
            << gz::math::Polynomial3d::Constant(-1.) << std::endl;

  // A cubic polynomial may be incomplete.
  const gz::math::Polynomial3d p(
      gz::math::Vector4d(1., 0., -1., 2.));
  std::cout << "A cubic polynomial may be incomplete: " << p << std::endl;

  // A polynomial can be evaluated.
  const double x = 0.5;
  std::cout << "Evaluating " << p << " at " << x
            << " yields " << p(x) << std::endl;

  // A polynomial can be queried for its minimum in a given interval,
  // as well as for its global minimum (which may not always be finite).
  const gz::math::Intervald interval =
      gz::math::Intervald::Closed(-1, 1.);
  std::cout << "The minimum of " << p << " in the " << interval
            << " interval is " << p.Minimum(interval) << std::endl;
  std::cout << "The global minimum of " << p
            << " is " << p.Minimum() << std::endl;

  const gz::math::Polynomial3d q(
      gz::math::Vector4d(0., 1., 2., 1.));
  std::cout << "The minimum of " << q << " in the " << interval
            << " interval is " << q.Minimum(interval) << std::endl;
  std::cout << "The global minimum of " << q
            << " is " << q.Minimum() << std::endl;

}
//! [complete]
