/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include <ignition/math.hh>

int main(int argc, char **argv)
{
  // Create a directed graph that is capable of storing integer data in the
  // vertices and double data on the edges.
  ignition::math::graph::DirectedGraph<int, double> graph(
    // Create the vertices, with default data and vertex ids.
    {
      {"vertex1"}, {"vertex2"}, {"vertex3"}
    },
    // Create the edges, with default data and weight values.
    {
      // Edge from vertex 0 to vertex 1. Each number refers to a vertex id.
      // Vertex ids start from zero.
      {{0, 1}}, {{1, 2}}
    });

  // Output in DOT graph format.
  std::cout << "Graph with default vertex Ids and data, "
            << "and default edge data.\n"
            << graph << std::endl;

  // You can assign data to vertices.
  ignition::math::graph::DirectedGraph<int, double> graph2(
    // Create the vertices, with custom data and default vertex ids.
    {
      {"vertex1", 1}, {"vertex2", 2}, {"vertex3", 10}
    },
    // Create the edges, with default data and weight values.
    {
      // Edge from vertex 0 to vertex 1. Each number refers to a vertex id
      // specified above.
      {{0, 2}}, {{1, 2}}
    });

  // Output in DOT graph format.
  std::cout << "Graph with default vertex Ids and custom vertex data, "
            << "and default edge data.\n"
            << graph2 << std::endl;

  // It's also possible to specify vertex ids.
  ignition::math::graph::DirectedGraph<int, double> graph3(
    // Create the vertices with custom data and vertex ids.
    {
      {"vertex1", 1, 2}, {"vertex2", 2, 3}, {"vertex3", 10, 4}
    },
    // Create the edges, with custom data and default weight values.
    {
      {{2, 3}, 6.3}, {{3, 4}, 4.2}
    });

  // Output in DOT graph format.
  std::cout << "Graph with custom vertex Ids and data, and custom edge data.\n"
            << graph3 << std::endl;

  // Finally, you can also assign weights to the edges.
  ignition::math::graph::DirectedGraph<int, double> graph4(
    // Create the vertices with custom data and vertex ids.
    {
      {"vertex1", 1, 2}, {"vertex2", 2, 3}, {"vertex3", 10, 4}
    },
    // Create the edges, with custom data and default weight values.
    {
      {{2, 3}, 6.3, 1.1}, {{3, 4}, 4.2, 2.3}
    });

  // Output in DOT graph format.
  std::cout << "Graph with custom vertex Ids and data, "
            << "and custom edge data and weights.\n"
            << graph4 << std::endl;
}
