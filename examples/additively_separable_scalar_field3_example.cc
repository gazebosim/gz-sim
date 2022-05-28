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
#include <gz/math/AdditivelySeparableScalarField3.hh>
#include <gz/math/Polynomial3.hh>

int main(int argc, char **argv)
{
  const double kConstant = 1.;
  const gz::math::Polynomial3d xPoly(
      gz::math::Vector4d(0., 1., 0., 1.));
  const gz::math::Polynomial3d yPoly(
      gz::math::Vector4d(1., 0., 1., 0.));
  const gz::math::Polynomial3d zPoly(
      gz::math::Vector4d(1., 0., 0., -1.));
  using AdditivelySeparableScalarField3dT =
      gz::math::AdditivelySeparableScalarField3d<
        gz::math::Polynomial3d>;
  const AdditivelySeparableScalarField3dT scalarField(
      kConstant, xPoly, yPoly, zPoly);

  // A printable, additively separable scalar field.
  std::cout << "An additively separable scalar field in R^3 "
            << "can be expressed as a sum of scalar functions "
            << "e.g. F(x, y, z) = " << scalarField << std::endl;

  // An additively separable scalar field can be evaluated.
  std::cout << "Evaluating F(x, y, z) at (0, 1, 0) yields "
            << scalarField(gz::math::Vector3d::UnitY)
            << std::endl;

  // An additively separable scalar field can be queried for its
  // minimum (provided the underlying scalar function allows it).
  std::cout << "The global minimum of F(x, y, z) is "
            << scalarField.Minimum() << std::endl;
}
//! [complete]
