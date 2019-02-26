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
#include <string>

#include "ignition/math/graph/Graph.hh"

using namespace ignition;
using namespace math;
using namespace graph;

// Define a test fixture class template.
template <class T>
class GraphTestFixture : public testing::Test
{
};

// Simulate a class with the maximum allowed number of vertices used.
template <typename V, typename E>
class MockVerticesFullUndirectedGraph : public UndirectedGraph<V, E>
{
  // Default constructor.
  public: MockVerticesFullUndirectedGraph()
  {
    this->nextVertexId = MAX_UI64;
  }
};

// Simulate a class with the maximum allowed number of edges used.
template <typename V, typename E>
class MockEdgesFullUndirectedGraph : public UndirectedGraph<V, E>
{
  // Default constructor.
  public: MockEdgesFullUndirectedGraph()
  {
    this->nextEdgeId = MAX_UI64;
  }
};

// The list of graphs we want to test.
using GraphTypes = ::testing::Types<DirectedGraph<int, double>,
                                    UndirectedGraph<int, double>>;
TYPED_TEST_CASE(GraphTestFixture, GraphTypes);

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, UniformInitialization)
{
  {
    TypeParam graph(
    {
      // Create vertices with custom Ids.
      {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
      // Create edges.
      {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
    });

    // Verify the vertices.
    auto vertices = graph.Vertices();
    EXPECT_EQ(3u, vertices.size());

    for (int i = 0; i < 3; ++i)
    {
      unsigned int iu = i;
      ASSERT_NE(vertices.find(i), vertices.end());
      auto v = vertices.at(i).get();
      EXPECT_EQ(std::to_string(i), v.Name());
      EXPECT_EQ(iu, v.Id());
      EXPECT_EQ(i, v.Data());
    }

    // Verify the edges.
    auto edges = graph.Edges();
    EXPECT_EQ(2u, edges.size());
  }
  {
    TypeParam graph(
    {
      // Create vertices with automatic Id selection.
      {{"0", 0}, {"1", 1}, {"2", 2}},
      // Create edges.
      {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
    });

    // Verify the vertices.
    auto vertices = graph.Vertices();
    EXPECT_EQ(3u, vertices.size());

    for (int i = 0; i < 3; ++i)
    {
      unsigned int iu = i;
      ASSERT_NE(vertices.find(i), vertices.end());
      auto v = vertices.at(i).get();
      EXPECT_EQ(std::to_string(i), v.Name());
      EXPECT_EQ(iu, v.Id());
      EXPECT_EQ(i, v.Data());
    }

    // Verify the edges.
    auto edges = graph.Edges();
    EXPECT_EQ(2u, edges.size());
  }
}

/////////////////////////////////////////////////
TEST(GraphTest, BadUniformInitializationMock)
{
  // There's no space for more vertices (mocked).
  {
    MockVerticesFullUndirectedGraph<int, double> graph;
    graph.AddVertex("0", 0);
    auto vertices = graph.Vertices();
    EXPECT_EQ(0u, vertices.size());
  }

  // There's no space for more edges (mocked).
  {
    MockEdgesFullUndirectedGraph<int, double> graph;
    graph.AddVertex("0", 0);
    graph.AddVertex("1", 1);
    graph.AddEdge({0, 1}, 1.0);
    auto edges = graph.Edges();
    EXPECT_EQ(0u, edges.size());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, BadUniformInitialization)
{
  // Bad graph initialization: repeated vertex Id.
  {
    TypeParam graph(
    {
      // Create vertices with custom Ids.
      {{"0", 0, 0}, {"1", 1, 0}, {"1", 1, 1}},
      // Create edges.
      {{{0, 1}, 0.0}}
    });

    // Verify the vertices.
    auto vertices = graph.Vertices();
    EXPECT_EQ(2u, vertices.size());

    for (int i = 0; i < 2; ++i)
    {
      unsigned int iu = i;
      ASSERT_NE(vertices.find(i), vertices.end());
      auto v = vertices.at(i).get();
      EXPECT_EQ(std::to_string(i), v.Name());
      EXPECT_EQ(iu, v.Id());
      EXPECT_EQ(i, v.Data());
    }
  }
  // Bad graph initialization: edges referencing an inexistent vertex.
  {
    TypeParam graph(
    {
      // Create vertices with custom Ids.
      {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
      // Create edges.
      {{{0, 3}, 0.0}}
    });

    // Verify the edges.
    auto edges = graph.Edges();
    EXPECT_EQ(0u, edges.size());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, VertexFromId)
{
  // Mutable version of VertexFromId().
  {
    TypeParam graph;

    // Create some vertices.
    auto &v0 = graph.AddVertex("0", 0, 0);
    EXPECT_EQ("0", v0.Name());
    graph.AddVertex("1", 1, 1);
    graph.AddVertex("2", 2, 2);

    auto v = graph.VertexFromId(v0.Id());
    EXPECT_EQ(v0.Id(), v.Id());

    // Id not found.
    v = graph.VertexFromId(500);
    EXPECT_EQ(kNullId, v.Id());
  }

  // Non-mutable version of VertexFromId().
  {
    const TypeParam graph(
    {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {}
    });

    auto v = graph.VertexFromId(0);
    EXPECT_EQ(0u, v.Id());

    // Id not found.
    v = graph.VertexFromId(500);
    EXPECT_EQ(kNullId, v.Id());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, Vertices)
{
  TypeParam graph(
  {
    {{"0", 10, 0}, {"1", 20, 1}, {"2", 30, 2}},
    {{{0, 1}, 0.0}, {{1, 2}, 0.0}}
  });

  auto vertices = graph.Vertices();
  EXPECT_EQ(3u, vertices.size());

  // Check that the vertex Ids start from 0.
  EXPECT_NE(vertices.end(), vertices.find(0));
  EXPECT_NE(vertices.end(), vertices.find(1));
  EXPECT_NE(vertices.end(), vertices.find(2));

  // Check the references.
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 0:
      {
        EXPECT_EQ("0", vertex.Name());
        EXPECT_EQ(10, vertex.Data());
        break;
      }
      case 1:
      {
        EXPECT_EQ("1", vertex.Name());
        EXPECT_EQ(20, vertex.Data());
        break;
      }
      case 2:
      {
        EXPECT_EQ("2", vertex.Name());
        EXPECT_EQ(30, vertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, VerticesNames)
{
  // Create a few vertices with two of them sharing the same name.
  TypeParam graph(
  {
    {{"vertex_0", 0}, {"vertex_1", 1}, {"common", 2}, {"common", 3}},
    {}
  });

  auto vertices = graph.Vertices("common");
  EXPECT_EQ(2u, vertices.size());

  // Check the Ids.
  EXPECT_NE(vertices.end(), vertices.find(2));
  EXPECT_NE(vertices.end(), vertices.find(3));

  // Check the references.
  for (auto const &vertexPair : vertices)
  {
    auto &vertex = vertexPair.second.get();
    switch (vertex.Id())
    {
      case 2:
      {
        EXPECT_EQ("common", vertex.Name());
        EXPECT_EQ(2, vertex.Data());
        break;
      }
      case 3:
      {
        EXPECT_EQ("common", vertex.Name());
        EXPECT_EQ(3, vertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, Empty)
{
  TypeParam graph;

  EXPECT_TRUE(graph.Empty());

  // Create a vertex.
  auto &v0 = graph.AddVertex("0", 0);
  ASSERT_TRUE(v0.Valid());
  EXPECT_FALSE(graph.Empty());
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, AddVertex)
{
  TypeParam graph;

  // Create some vertices without Id.
  auto &v0 = graph.AddVertex("0", 0);
  EXPECT_TRUE(v0.Id() != kNullId);
  auto &v1 = graph.AddVertex("1", 1);
  EXPECT_TRUE(v1.Id() != kNullId);
  auto &v2 = graph.AddVertex("2", 2);
  EXPECT_TRUE(v2.Id() != kNullId);

  // Create a vertex with Id.
  auto &v3 = graph.AddVertex("3", 5, 3);
  EXPECT_EQ(3u, v3.Id());
  EXPECT_EQ(5, v3.Data());
  EXPECT_EQ("3", v3.Name());

  // Create a vertex with an already used Id.
  auto &v4 = graph.AddVertex("3", 0, 3);
  ASSERT_TRUE(v4.Id() == kNullId);

  auto vertices = graph.Vertices();
  EXPECT_EQ(4u, vertices.size());

  // Change data in v3 and verify that is propagated into the graph.
  v3.Data() = 10;
  auto &vertex = graph.VertexFromId(v3.Id());
  EXPECT_EQ(10, vertex.Data());

  // Try to change data in v4 and verify that is not propagated into the graph.
  v4.Data() = 20;
  for (auto const &vertexPair : vertices)
  {
    auto &v = vertexPair.second.get();
    EXPECT_NE(20, v.Data());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgeFromVertices)
{
  TypeParam graph(
      {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {{{0, 1}, 2.0}, {{1, 2}, 3}}
      });

  const auto &edge = graph.EdgeFromVertices(0, 1);
  EXPECT_NE(kNullId, edge.Id());
  EXPECT_DOUBLE_EQ(2.0, edge.Data());

  {
    const auto &noEdge = graph.EdgeFromVertices(0, 2);
    EXPECT_EQ(kNullId, noEdge.Id());
  }

  {
    const auto &noEdge = graph.EdgeFromVertices(4, 5);
    EXPECT_EQ(kNullId, noEdge.Id());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgeFromId)
{
  // Mutable version of EdgeFromId().
  {
    TypeParam graph(
    {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {{{0, 1}, 2.0}, {{1, 2}, 3.0}}
    });

    auto e = graph.EdgeFromId(1);
    EXPECT_EQ(1u, e.Id());

    // Id not found.
    e = graph.EdgeFromId(500);
    EXPECT_EQ(kNullId, e.Id());
  }

  // Non-mutable version of EdgeFromId().
  {
    const TypeParam graph(
    {
      {{"0", 0}, {"1", 1}, {"2", 2}},
      {{{0, 1}, 2.0}, {{1, 2}, 3.0}}
    });

    auto e = graph.EdgeFromId(1);
    EXPECT_EQ(1u, e.Id());

    // Id not found.
    e = graph.EdgeFromId(500);
    EXPECT_EQ(kNullId, e.Id());
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgelessInDegree)
{
  TypeParam graph;

  // add a bunch of vertices but no edges
  const int vertexCount = 10000;
  for (int i = 0; i < vertexCount; ++i)
  {
    auto &v = graph.AddVertex(std::to_string(i), i);
    EXPECT_TRUE(v.Valid());
  }

  for (auto const &idVertex : graph.Vertices())
  {
    EXPECT_EQ(0u, graph.InDegree(idVertex.first));
  }
}

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, EdgelessOutDegree)
{
  TypeParam graph;

  // add a bunch of vertices but no edges
  const int vertexCount = 10000;
  for (int i = 0; i < vertexCount; ++i)
  {
    auto &v = graph.AddVertex(std::to_string(i), i);
    EXPECT_TRUE(v.Valid());
  }

  for (auto const &idVertex : graph.Vertices())
  {
    EXPECT_EQ(0u, graph.OutDegree(idVertex.first));
  }
}
