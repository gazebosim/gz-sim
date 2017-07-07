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

#include "ignition/math/graph/Edge.hh"
#include "ignition/math/graph/Vertex.hh"

using namespace ignition;
using namespace math;
using namespace graph;

// Define a test fixture class template.
template <class T>
class EdgeTestFixture : public testing::Test
{
};

// The list of edges we want to test.
using EdgeTypes = ::testing::Types<DirectedEdge<int>,
                                   UndirectedEdge<int>>;
TYPED_TEST_CASE(EdgeTestFixture, EdgeTypes);

/////////////////////////////////////////////////
TYPED_TEST(EdgeTestFixture, Accessors)
{
  {
    EdgeId id = 1;
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    TypeParam edge(id, vertices, data, weight);

    EXPECT_EQ(edge.Id(), id);
    EXPECT_DOUBLE_EQ(edge.Weight(), weight);
    EXPECT_EQ(edge.Vertices(), vertices);
    EXPECT_EQ(edge.Data(), data);
    // Modify the data.
    edge.Data() += 1;
    EXPECT_EQ(edge.Data(), data + 1);
    EXPECT_TRUE(edge.Valid());
  }

  {
    EdgeId id = kNullId;
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    TypeParam edge(id, vertices, data, weight);

    EXPECT_EQ(edge.Id(), id);
    EXPECT_DOUBLE_EQ(edge.Weight(), weight);
    VertexId_P expectedVertices = {kNullId, kNullId};
    EXPECT_EQ(edge.Vertices(), expectedVertices);
    EXPECT_EQ(edge.Data(), data);
    // Modify the data.
    edge.Data() += 1;
    EXPECT_EQ(edge.Data(), data + 1);
    EXPECT_FALSE(edge.Valid());
  }
}

/////////////////////////////////////////////////
TEST(EdgeTest, FromToDirected)
{
  {
    EdgeId id = 1;
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    DirectedEdge<int> edge(id, vertices, data, weight);

    EXPECT_EQ(edge.From(0), 1);
    EXPECT_EQ(edge.To(1), 0);

    // It's a directed edge, you cannot go in this direction.
    EXPECT_EQ(edge.From(1), kNullId);
    EXPECT_EQ(edge.To(0), kNullId);

    // The Id doesn't exit.
    EXPECT_EQ(edge.From(99), kNullId);
    EXPECT_EQ(edge.To(99), kNullId);
  }

  {
    EdgeId id = kNullId;
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    DirectedEdge<int> edge(id, vertices, data, weight);
    // The edge is not valid because the Id == kNullId.
    EXPECT_FALSE(edge.Valid());

    EXPECT_EQ(edge.From(0), kNullId);
    EXPECT_EQ(edge.To(1), kNullId);
    EXPECT_EQ(edge.From(1), kNullId);
    EXPECT_EQ(edge.To(0), kNullId);
  }
}

/////////////////////////////////////////////////
TEST(EdgeTest, FromToUndirected)
{
  {
    EdgeId id = 1;
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    UndirectedEdge<int> edge(id, vertices, data, weight);

    EXPECT_EQ(edge.From(0), 1);
    EXPECT_EQ(edge.To(1), 0);
    EXPECT_EQ(edge.From(1), 0);
    EXPECT_EQ(edge.To(0), 1);

    // The Id doesn't exit.
    EXPECT_EQ(edge.From(99), kNullId);
    EXPECT_EQ(edge.To(99), kNullId);
  }

  {
    EdgeId id = kNullId;
    double weight = 2.0;
    // Only one vertex.
    VertexId_P vertices = {0, 1};
    int data = 3;
    UndirectedEdge<int> edge(id, vertices, data, weight);
    // The edge is not valid because the Id == kNullId.
    EXPECT_FALSE(edge.Valid());

    EXPECT_EQ(edge.From(0), kNullId);
    EXPECT_EQ(edge.To(1), kNullId);
    EXPECT_EQ(edge.From(1), kNullId);
    EXPECT_EQ(edge.To(0), kNullId);
  }
}

/////////////////////////////////////////////////
TEST(EdgeTest, StreamInsertionDirected)
{
  EdgeId id = 1;
  double weight = 2.0;
  VertexId_P vertices = {0, 1};
  int data = 3;
  DirectedEdge<int> edge(id, vertices, data, weight);

  std::ostringstream output;
  output << edge;

  std::string expectedOutput = "  0 -> 1 [label=2];\n";
  EXPECT_EQ(output.str(), expectedOutput);
}

/////////////////////////////////////////////////
TEST(EdgeTest, StreamInsertionUndirected)
{
  EdgeId id = 1;
  double weight = 2.0;
  VertexId_P vertices = {0, 1};
  int data = 3;
  UndirectedEdge<int> edge(id, vertices, data, weight);

  std::ostringstream output;
  output << edge;

  std::string expectedOutput = "  0 -- 1 [label=2];\n";
  EXPECT_EQ(output.str(), expectedOutput);
}
