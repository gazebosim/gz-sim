/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <iostream>
#include <gz/math.hh>

int main(int argc, char **argv)
{
  // Create a triangle with the following vertices:
  // 1: x = -1, y = 0
  // 2: x = 0, y = 1
  // 3: x = 1, y = 0
//! [constructor]
  gz::math::Triangled tri(
      gz::math::Vector2d(-1, 0),
      gz::math::Vector2d(0, 1),
      gz::math::Vector2d(1, 0));
//! [constructor]

  // The individual vertices are accessible through the [] operator

//! [access1]
  std::cout << "Vertex 1: " << tri[0] << "\n"
            << "Vertex 2: " << tri[1] << "\n"
            << "Vertex 3: " << tri[2] << "\n";
//! [access1]

  // Each side of the triangle is also accessible via the Side function
//! [access2]
  std::cout << "Side 1: " << tri.Side(0) << "\n"
            << "Side 2: " << tri.Side(1) << "\n"
            << "Side 3: " << tri.Side(2) << "\n";
//! [access2]

  // It's also possible to set each vertex individually.
//! [vertex1]
  tri.Set(0, gz::math::Vector2d(-10, 0));
  tri.Set(1, gz::math::Vector2d(0, 20));
  tri.Set(2, gz::math::Vector2d(10, 2));
//! [vertex1]

  // Or set all the vertices at once.
//! [vertex2]
  tri.Set(gz::math::Vector2d(-1, 0),
          gz::math::Vector2d(0, 1),
          gz::math::Vector2d(1, 0));
//! [vertex2]

  // You can get the perimeter length and area of the triangle
//! [perimeter and area]
  std::cout << "Perimeter=" << tri.Perimeter()
            << " Area=" << tri.Area() << "\n";
//! [perimeter and area]

  // The Contains functions check if a line or point is inside the triangle
//! [contains]
  if (tri.Contains(gz::math::Vector2d(0, 0.5)))
    std::cout << "Triangle contains the point 0, 0.5\n";
  else
    std::cout << "Triangle does not contain the point 0, 0.5\n";
//! [contains]

  // The Intersect function check if a line segment intersects the triangle.
  // It also returns the points of intersection
//! [intersect]
  gz::math::Vector2d pt1, pt2;
  if (tri.Intersects(gz::math::Line2d(-2, 0.5, 2, 0.5), pt1, pt2))
  {
    std::cout << "A line from (-2, 0.5) to (2, 0.5) intersects "
              << "the triangle at the\nfollowing points:\n"
              << "\t Pt1=" << pt1 << "\n"
              << "\t Pt2=" << pt2 << "\n";
  }
  else
  {
    std::cout << "A line from (-2, 0.5) to (2, 0.5) does not intersect "
              << "the triangle\n";
  }
//! [intersect]

  // There are more functions in Triangle. Take a look at the API;
  // http://gazebosim.org/libraries/ign_mat/api
}
