/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <gz/math/Color.hh>

int main(int argc, char **argv) 
{

//! [Create a color]
  gz::math::Color a = gz::math::Color(0.3, 0.4, 0.5);
//! [Create a color]
  // The channel order is R, G, B, A and the default alpha value of a should be 1.0
  std::cout << "The alpha value of a should be 1: " << a.A() << std::endl;
  
  
//! [Set a new color value]
  a.gz::math::Color::Set(0.6, 0.7, 0.8, 1.0);
//! [Set a new color value]
  std::cout << "The RGBA value of a: " << a << std::endl;

//! [Set value from BGRA]
  // 0xFF0000FF is blue in BGRA. Convert it to RGBA.
  gz::math::Color::BGRA blue = 0xFF0000FF;
  a.gz::math::Color::SetFromBGRA(blue);
//! [Set value from BGRA]

//! [Math operator]
  std::cout << "Check if a is Blue: " << (a == gz::math::Color::Blue) << std::endl;
  std::cout << "The RGB value of a should be (0, 0, 1): " << a[0] << ", " 
                                                          << a[1] << ", " 
                                                          << a[2] << std::endl;
//! [Math operator]

//! [Set value from HSV]
  // Initialize with HSV. (240, 1.0, 1.0) is blue in HSV.
  a.gz::math::Color::SetFromHSV(240.0, 1.0, 1.0);
  std::cout << "The HSV value of a: " << a.HSV() << std::endl;
  std::cout << "The RGBA value of a should be (0, 0, 1, 1): " << a << std::endl;
//! [Set value from HSV]

}
//! [complete]
