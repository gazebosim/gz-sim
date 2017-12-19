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
    double newWeight = 3.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    TypeParam edge(vertices, data, weight, id);

    // Id.
    EXPECT_EQ(id, edge.Id());

    // Weight.
    EXPECT_DOUBLE_EQ(weight, edge.Weight());
    edge.SetWeight(newWeight);
    EXPECT_DOUBLE_EQ(newWeight, edge.Weight());

    // Vertices.
    EXPECT_EQ(vertices, edge.Vertices());

    // Data.
    EXPECT_EQ(data, edge.Data());
    edge.Data() += 1;
    EXPECT_EQ(data + 1, edge.Data());

    // Validation.
    EXPECT_TRUE(edge.Valid());
  }

  {
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    TypeParam edge(vertices, data, weight);

    // Id.
    EXPECT_EQ(kNullId, edge.Id());

    // Weight.
    EXPECT_DOUBLE_EQ(weight, edge.Weight());

    // Vertices.
    VertexId_P expectedVertices = {kNullId, kNullId};
    EXPECT_EQ(expectedVertices, edge.Vertices());

    // Data.
    EXPECT_EQ(data, edge.Data());
    edge.Data() += 1;
    EXPECT_EQ(data + 1, edge.Data());

    // Validation. It does not have a valid Id.
    EXPECT_FALSE(edge.Valid());
  }
}

/////////////////////////////////////////////////
TEST(EdgeTest, Initializer)
{
  VertexId_P vertices = {1, 2};
  std::string data = "hi";

  {
    double weight = 2.0;
    EdgeInitializer<std::string> edgeInitializer(vertices, data, weight);
    EXPECT_EQ(vertices, edgeInitializer.vertices);
    EXPECT_EQ(data, edgeInitializer.data);
    EXPECT_DOUBLE_EQ(weight, edgeInitializer.weight);
  }
  {
    EdgeInitializer<std::string> edgeInitializer(vertices, "hi");
    EXPECT_EQ(vertices, edgeInitializer.vertices);
    EXPECT_EQ("hi", edgeInitializer.data);
    EXPECT_DOUBLE_EQ(1.0, edgeInitializer.weight);
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
    DirectedEdge<int> edge(vertices, data, weight, id);

    EXPECT_EQ(0u, edge.Tail());
    EXPECT_EQ(1u, edge.Head());

    EXPECT_EQ(edge.Head(), edge.From(edge.Tail()));
    EXPECT_EQ(edge.Tail(), edge.To(edge.Head()));

    // It's a directed edge, you cannot go in this direction.
    EXPECT_EQ(kNullId, edge.From(edge.Head()));
    EXPECT_EQ(kNullId, edge.To(edge.Tail()));

    // The Id doesn't exit.
    EXPECT_EQ(kNullId, edge.From(99));
    EXPECT_EQ(kNullId, edge.To(99));
  }

  {
    double weight = 2.0;
    VertexId_P vertices = {0, 1};
    int data = 3;
    DirectedEdge<int> edge(vertices, data, weight);
    // The edge is not valid because the Id == kNullId.
    EXPECT_FALSE(edge.Valid());

    EXPECT_EQ(kNullId, edge.From(edge.Tail()));
    EXPECT_EQ(kNullId, edge.To(edge.Head()));
    EXPECT_EQ(kNullId, edge.From(edge.Head()));
    EXPECT_EQ(kNullId, edge.To(edge.Tail()));
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
    UndirectedEdge<int> edge(vertices, data, weight, id);

    EXPECT_EQ(1u, edge.From(0));
    EXPECT_EQ(0u, edge.To(1));
    EXPECT_EQ(0u, edge.From(1));
    EXPECT_EQ(1u, edge.To(0));

    // The Id doesn't exit.
    EXPECT_EQ(kNullId, edge.From(99));
    EXPECT_EQ(kNullId, edge.To(99));
  }

  {
    EdgeId id = kNullId;
    double weight = 2.0;
    // Only one vertex.
    VertexId_P vertices = {0, 1};
    int data = 3;
    UndirectedEdge<int> edge(vertices, data, weight, id);
    // The edge is not valid because the Id == kNullId.
    EXPECT_FALSE(edge.Valid());

    EXPECT_EQ(kNullId, edge.From(0));
    EXPECT_EQ(kNullId, edge.To(1));
    EXPECT_EQ(kNullId, edge.From(1));
    EXPECT_EQ(kNullId, edge.To(0));
  }
}

/////////////////////////////////////////////////
TEST(EdgeTest, StreamInsertionDirected)
{
  EdgeId id = 1;
  double weight = 2.0;
  VertexId_P vertices = {0, 1};
  int data = 3;
  DirectedEdge<int> edge(vertices, data, weight, id);

  std::ostringstream output;
  output << edge;

  std::string expectedOutput = "  0 -> 1 [label=2];\n";
  EXPECT_EQ(expectedOutput, output.str());
}

/////////////////////////////////////////////////
TEST(EdgeTest, StreamInsertionUndirected)
{
  EdgeId id = 1;
  double weight = 2.0;
  VertexId_P vertices = {0, 1};
  int data = 3;
  UndirectedEdge<int> edge(vertices, data, weight, id);

  std::ostringstream output;
  output << edge;

  std::string expectedOutput = "  0 -- 1 [label=2];\n";
  EXPECT_EQ(expectedOutput, output.str());
}
