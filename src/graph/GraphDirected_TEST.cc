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

#include "ignition/math/graph/Graph.hh"

using namespace ignition;
using namespace math;
using namespace graph;

/////////////////////////////////////////////////
TEST(GraphTest, Edges)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  auto edges = graph.Edges();
  EXPECT_EQ(4u, edges.size());

  // Check the Ids.
  for (auto i = 0; i < 4; ++i)
    EXPECT_NE(edges.end(), edges.find(i));

  // Check the references.
  for (auto const &edgePair : edges)
  {
    auto &edge = edgePair.second.get();
    switch (edge.Id())
    {
      case 0:
      {
        EXPECT_EQ(0u, edge.Tail());
        EXPECT_EQ(0u, edge.Head());
        EXPECT_DOUBLE_EQ(1.0, edge.Data());
        break;
      }
      case 1:
      {
        EXPECT_EQ(0u, edge.Tail());
        EXPECT_EQ(1u, edge.Head());
        EXPECT_DOUBLE_EQ(2.0, edge.Data());
        break;
      }
      case 2:
      {
        EXPECT_EQ(1u, edge.Tail());
        EXPECT_EQ(2u, edge.Head());
        EXPECT_DOUBLE_EQ(3.0, edge.Data());
        break;
      }
      case 3:
      {
        EXPECT_EQ(2u, edge.Tail());
        EXPECT_EQ(0u, edge.Head());
        EXPECT_DOUBLE_EQ(4.0, edge.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TEST(GraphTest, AdjacentsFrom)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  // Try to get the adjacents from an inexistent vertex.
  auto adjacents = graph.AdjacentsFrom(kNullId);
  EXPECT_TRUE(adjacents.empty());

  adjacents = graph.AdjacentsFrom(0);
  EXPECT_EQ(2u, adjacents.size());
  EXPECT_NE(adjacents.end(), adjacents.find(0));
  EXPECT_NE(adjacents.end(), adjacents.find(1));

  auto vertex = graph.VertexFromId(0);
  adjacents = graph.AdjacentsFrom(vertex);
  EXPECT_EQ(2u, adjacents.size());
  EXPECT_NE(adjacents.end(), adjacents.find(0));
  EXPECT_NE(adjacents.end(), adjacents.find(1));

  // Check the references.
  for (auto const &vertexPair : adjacents)
  {
    auto &neighborVertex = vertexPair.second.get();
    switch (neighborVertex.Id())
    {
      case 0:
      {
        EXPECT_EQ("0", neighborVertex.Name());
        EXPECT_EQ(0, neighborVertex.Data());
        break;
      }
      case 1:
      {
        EXPECT_EQ("1", neighborVertex.Name());
        EXPECT_EQ(1, neighborVertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TEST(GraphTest, AdjacentsTo)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v2), (v2-->v1)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 1}, 4.0}}
  });

  // Try to get the adjacents to an inexistent vertex.
  auto adjacents = graph.AdjacentsTo(kNullId);
  EXPECT_TRUE(adjacents.empty());

  adjacents = graph.AdjacentsTo(0);
  EXPECT_EQ(1u, adjacents.size());
  EXPECT_NE(adjacents.end(), adjacents.find(0));

  adjacents = graph.AdjacentsTo(1);
  EXPECT_EQ(2u, adjacents.size());
  EXPECT_NE(adjacents.end(), adjacents.find(0));
  EXPECT_NE(adjacents.end(), adjacents.find(2));

  auto vertex = graph.VertexFromId(2);
  adjacents = graph.AdjacentsTo(vertex);
  EXPECT_EQ(1u, adjacents.size());
  EXPECT_NE(adjacents.end(), adjacents.find(1));

  // Check the references.
  for (auto const &vertexPair : adjacents)
  {
    auto &neighborVertex = vertexPair.second.get();
    switch (neighborVertex.Id())
    {
      case 1:
      {
        EXPECT_EQ("1", neighborVertex.Name());
        EXPECT_EQ(1, neighborVertex.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TEST(GraphTest, IncidentsFrom)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v0), (v1-->v2)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 0}, 3.0}, {{1, 2}, 4.0}}
  });

  auto incidents = graph.IncidentsFrom(0);
  EXPECT_EQ(2u, incidents.size());
  EXPECT_NE(incidents.end(), incidents.find(0));
  EXPECT_NE(incidents.end(), incidents.find(1));

  auto vertex = graph.VertexFromId(1);
  incidents = graph.IncidentsFrom(vertex);
  EXPECT_EQ(2u, incidents.size());
  EXPECT_NE(incidents.end(), incidents.find(2));
  EXPECT_NE(incidents.end(), incidents.find(3));

  // Check the references.
  for (auto const &edgePair : incidents)
  {
    auto &edge = edgePair.second.get();
    switch (edge.Id())
    {
      case 2:
      {
        auto vertices = edge.Vertices();
        EXPECT_EQ(1u, vertices.first);
        EXPECT_EQ(0u, vertices.second);
        EXPECT_DOUBLE_EQ(3.0, edge.Data());
        break;
      }
      case 3:
      {
        auto vertices = edge.Vertices();
        EXPECT_EQ(1u, vertices.first);
        EXPECT_EQ(2u, vertices.second);
        EXPECT_DOUBLE_EQ(4.0, edge.Data());
        break;
      }
      default:
        FAIL();
    };
  }

  incidents = graph.IncidentsFrom(2);
  EXPECT_TRUE(incidents.empty());

  // Try an inexistent vertex.
  incidents = graph.IncidentsFrom(kNullId);
  EXPECT_EQ(0u, incidents.size());
}

/////////////////////////////////////////////////
TEST(GraphTest, IncidentsTo)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  auto incidents = graph.IncidentsTo(0);
  EXPECT_EQ(2u, incidents.size());
  EXPECT_NE(incidents.end(), incidents.find(0));
  EXPECT_NE(incidents.end(), incidents.find(3));

  auto vertex = graph.VertexFromId(0);
  incidents = graph.IncidentsTo(vertex);
  EXPECT_EQ(2u, incidents.size());
  EXPECT_NE(incidents.end(), incidents.find(0));
  EXPECT_NE(incidents.end(), incidents.find(3));

  // Check the references.
  for (auto const &edgePair : incidents)
  {
    auto &edge = edgePair.second.get();
    switch (edge.Id())
    {
      case 0:
      {
        auto vertices = edge.Vertices();
        EXPECT_EQ(0u, vertices.first);
        EXPECT_EQ(0u, vertices.second);
        EXPECT_DOUBLE_EQ(1.0, edge.Data());
        break;
      }
      case 3:
      {
        auto vertices = edge.Vertices();
        EXPECT_EQ(2u, vertices.first);
        EXPECT_EQ(0u, vertices.second);
        EXPECT_DOUBLE_EQ(4.0, edge.Data());
        break;
      }
      default:
        FAIL();
    };
  }
}

/////////////////////////////////////////////////
TEST(GraphTest, InDegree)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v2), (v2-->v1) x 2]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 1}, 4.0}, {{2, 1}, 4.0}}
  });

  EXPECT_EQ(1u, graph.InDegree(0));
  EXPECT_EQ(1u, graph.InDegree(graph.VertexFromId(0)));
  EXPECT_EQ(3u, graph.InDegree(1));
  EXPECT_EQ(3u, graph.InDegree(graph.VertexFromId(1)));
  EXPECT_EQ(1u, graph.InDegree(2));
  EXPECT_EQ(1u, graph.InDegree(graph.VertexFromId(2)));
}

/////////////////////////////////////////////////
TEST(GraphTest, OutDegree)
{
  // Create a graph with edges [(v0-->v0), (v0-->v1), (v1-->v0), (v1-->v2) x 2]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 0}, 1.0}, {{0, 1}, 2.0}, {{1, 0}, 3.0}, {{1, 2}, 4.0}, {{1, 2}, 5.0}}
  });

  EXPECT_EQ(2u, graph.OutDegree(0));
  EXPECT_EQ(2u, graph.OutDegree(graph.VertexFromId(0)));
  EXPECT_EQ(3u, graph.OutDegree(1));
  EXPECT_EQ(3u, graph.OutDegree(graph.VertexFromId(1)));
  EXPECT_EQ(0u, graph.OutDegree(2));
  EXPECT_EQ(0u, graph.OutDegree(graph.VertexFromId(2)));
}

/////////////////////////////////////////////////
TEST(GraphTest, AddEdge)
{
  // Create a graph with three vertices.
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {}
  });

  // Create some edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  auto &e0 = graph.AddEdge({0, 1}, 2.0);
  auto &e1 = graph.AddEdge({1, 2}, 3.0);
  auto &e2 = graph.AddEdge({2, 0}, 4.0);

  // Check the edge content.
  EXPECT_DOUBLE_EQ(2.0, e0.Data());
  EXPECT_DOUBLE_EQ(3.0, e1.Data());
  EXPECT_DOUBLE_EQ(4.0, e2.Data());

  // Change some content and verity it.
  e2.Data() = 5.0;
  e2.SetWeight(6.0);
  auto edge = graph.EdgeFromId(e2.Id());
  EXPECT_DOUBLE_EQ(5.0, edge.Data());
  EXPECT_DOUBLE_EQ(6.0, edge.Weight());

  // Check that the edges point to the right vertices.
  EXPECT_EQ(0u, e0.Tail());
  EXPECT_EQ(1u, e0.Head());

  auto edges = graph.Edges();
  EXPECT_EQ(3u, edges.size());

  // Try to add an edge with an incorrect tail.
  edge = graph.AddEdge({kNullId, 1}, 2.0);
  EXPECT_EQ(kNullId, edge.Id());
  EXPECT_EQ(3u, graph.Edges().size());

  // Try to add an edge with an incorrect head.
  edge = graph.AddEdge({0, kNullId}, 2.0);
  EXPECT_EQ(kNullId, edge.Id());
  EXPECT_EQ(3u, graph.Edges().size());
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveEdge)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  // Remove a nonexistent edge shouldn't cause any effect.
  EXPECT_FALSE(graph.RemoveEdge(kNullId));
  EXPECT_EQ(3u, graph.Edges().size());
  EXPECT_EQ(1u, graph.IncidentsTo(1).size());

  // Remove the edge (v0-->v1)
  EXPECT_TRUE(graph.RemoveEdge(0));
  EXPECT_EQ(2u, graph.Edges().size());
  EXPECT_EQ(0u, graph.IncidentsTo(1).size());

  // Remove the edge (v1-->v2)
  auto edge = graph.EdgeFromId(1);
  EXPECT_TRUE(graph.RemoveEdge(edge));
  EXPECT_EQ(1u, graph.Edges().size());

  // Try to remove an edge that doesn't exist anymore.
  EXPECT_FALSE(graph.RemoveEdge(1));
  EXPECT_EQ(1u, graph.Edges().size());

  // Remove the edge (v2-->v0)
  EXPECT_TRUE(graph.RemoveEdge(2));
  EXPECT_EQ(0u, graph.Edges().size());
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveVertex)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  // Remove a nonexistent vertex shouldn't cause any effect.
  EXPECT_FALSE(graph.RemoveVertex(kNullId));
  EXPECT_EQ(3u, graph.Vertices().size());
  EXPECT_EQ(1u, graph.AdjacentsFrom(1).size());

  // Remove vertex #2.
  EXPECT_TRUE(graph.RemoveVertex(2));
  EXPECT_EQ(2u, graph.Vertices().size());
  EXPECT_EQ(1u, graph.Edges().size());
  EXPECT_EQ(0u, graph.AdjacentsFrom(1).size());

  // Remove vertex #1.
  auto vertex = graph.VertexFromId(1);
  EXPECT_TRUE(graph.RemoveVertex(vertex));
  EXPECT_EQ(1u, graph.Vertices().size());
  EXPECT_TRUE(graph.Edges().empty());

  // Try to remove a vertex (#1) that doesn't exist anymore.
  EXPECT_FALSE(graph.RemoveVertex(1));
  EXPECT_EQ(1u, graph.Vertices().size());
  EXPECT_TRUE(graph.Edges().empty());

  // Remove vertex #0.
  EXPECT_TRUE(graph.RemoveVertex(0));
  EXPECT_TRUE(graph.Vertices().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, RemoveVertices)
{
  // Create a graph with edges [(v0-->v1), (v1-->v2), (v2-->v3), (v3-->v0)]
  DirectedGraph<int, double> graph(
  {
    {{"v0", 0, 0}, {"v1", 1, 1}, {"common", 2, 2}, {"common", 3, 3}},
    {{{0, 1}, 2.0}, {{1, 2}, 3.0}, {{2, 3}, 4.0}, {{3, 0}, 5.0}}
  });

  // Try to remove a node with a name that doesn't exist.
  EXPECT_EQ(graph.RemoveVertices("wrong_name"), 0u);
  EXPECT_EQ(4u, graph.Vertices().size());
  EXPECT_EQ(1u, graph.AdjacentsFrom(1).size());

  // Remove two vertices at the same time.
  EXPECT_EQ(2u, graph.RemoveVertices("common"));
  EXPECT_EQ(2u, graph.Vertices().size());
  EXPECT_EQ(1u, graph.Edges().size());

  EXPECT_EQ(0u, graph.AdjacentsFrom(1).size());

  EXPECT_EQ(1u, graph.RemoveVertices("v1"));
  EXPECT_EQ(1u, graph.Vertices().size());
  EXPECT_TRUE(graph.Edges().empty());

  EXPECT_EQ(1u, graph.RemoveVertices("v0"));
  EXPECT_TRUE(graph.Vertices().empty());

  EXPECT_TRUE(graph.Empty());
}

/////////////////////////////////////////////////
TEST(GraphTest, StreamInsertion)
{
  // Create a graph with 4 vertices and
  // edges [(v0-->v0), (v0-->v1), (v1-->v2), (v2-->v3)]
  DirectedGraph<int, double> graph(
  {
    {{"v0", 0, 0}, {"v1", 1, 1}, {"v2", 2, 2}, {"v3", 3, 3}},
    {{{0, 1}, 2.0, 4.0}, {{0, 0}, 2.0, 6.0}, {{1, 2}, 3.0}, {{2, 0}, 4.0}}
  });

  std::ostringstream output;
  output << graph;

  std::cout << "# Use this snippet with your favorite DOT tool." << std::endl;
  std::cout << graph << std::endl;

  for (auto const &s : {"digraph {\n",
                        "  0 [label=\"v0 (0)\"];\n",
                        "  1 [label=\"v1 (1)\"];\n",
                        "  2 [label=\"v2 (2)\"];\n",
                        "  3 [label=\"v3 (3)\"];\n",
                        "  0 -> 1 [label=4];\n",
                        "  0 -> 0 [label=6];\n",
                        "  1 -> 2 [label=1];\n",
                        "  2 -> 0 [label=1];\n"})
  {
    EXPECT_NE(std::string::npos, output.str().find(s));
  }
}
