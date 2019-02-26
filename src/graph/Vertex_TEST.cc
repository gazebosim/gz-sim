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

#include <gtest/gtest.h>
#include <iostream>
#include <string>

#include "ignition/math/graph/Vertex.hh"

using namespace ignition;
using namespace math;
using namespace graph;

/////////////////////////////////////////////////
TEST(VertexTest, Accessors)
{
  {
    int data = 5;
    Vertex<int> vertex("", data);
    EXPECT_TRUE(vertex.Name().empty());
    vertex.SetName("new_name");
    EXPECT_EQ("new_name", vertex.Name());
    EXPECT_EQ(vertex.Data(), data);
    EXPECT_EQ(vertex.Id(), kNullId);
    EXPECT_FALSE(vertex.Valid());
  }

  {
    std::string name = "my_vertex";
    int data = 10;
    VertexId id = 2;
    Vertex<int> vertex(name, data, id);
    EXPECT_EQ(vertex.Name(), name);
    EXPECT_EQ(vertex.Id(), id);
    EXPECT_EQ(vertex.Data(), data);
    // Modify the data
    vertex.Data() += 1;
    EXPECT_EQ(vertex.Data(), data + 1);
    EXPECT_TRUE(vertex.Valid());
  }
}

/////////////////////////////////////////////////
TEST(VertexTest, StreamInsertion)
{
  std::string name = "my_vertex";
  int data = 10;
  VertexId id = 2;
  Vertex<int> vertex(name, data, id);

  std::ostringstream output;
  output << vertex;

  std::string expectedOutput = "  " + std::to_string(id) + " [label=\"" + name +
    " (" + std::to_string(id) + ")\"];\n";
  EXPECT_EQ(output.str(), expectedOutput);
}
