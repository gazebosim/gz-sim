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
#include "ignition/math/graph/GraphAlgorithms.hh"

using namespace ignition;
using namespace math;
using namespace graph;

// Define a test fixture class template.
template <class T>
class GraphTestFixture : public testing::Test
{
};

// The list of graphs we want to test.
using GraphTypes = ::testing::Types<DirectedGraph<int, double>,
                                    UndirectedGraph<int, double>>;
TYPED_TEST_CASE(GraphTestFixture, GraphTypes);

/////////////////////////////////////////////////
TYPED_TEST(GraphTestFixture, BreadthFirstSort)
{
  TypeParam graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  auto res = BreadthFirstSort(graph, 0);
  std::vector<VertexId> expected = {0, 1, 2, 4, 3, 5, 6};
  EXPECT_EQ(expected, res);
}

/////////////////////////////////////////////////
TEST(GraphTest, DepthFirstSortDirected)
{
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0}, {{1, 3}, 2.0},
     {{1, 5}, 3.0}, {{2, 6}, 4.0}, {{5, 4}, 2.0}}
  });

  auto res = DepthFirstSort(graph, 0);
  std::vector<VertexId> expected = {0, 4, 2, 6, 1, 5, 3};
  EXPECT_EQ(expected, res);
}

/////////////////////////////////////////////////
TEST(UndirectedGraphTest, DepthFirstSortUndirected)
{
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0}, {{1, 3}, 2.0},
     {{1, 5}, 3.0}, {{2, 6}, 4.0}, {{5, 4}, 2.0}}
  });

  auto res = DepthFirstSort(graph, 0);
  std::vector<VertexId> expected = {0, 4, 5, 1, 3, 2, 6};
  EXPECT_EQ(expected, res);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, DijkstraUndirected)
{
  ///              (6)
  ///           0-------1
  ///           |      /|\
  ///           |     / | \(5)
  ///           | (2)/  |  \
  ///           |   /   |   2
  ///        (1)|  / (2)|  /
  ///           | /     | /(5)
  ///           |/      |/
  ///           3-------4
  ///              (1)
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}, {"3", 3, 3}, {"4", 4, 4}},
    // Edges.
    {{{0, 1}, 2.0, 6.0}, {{0, 3}, 3.0, 1.0},
     {{1, 2}, 4.0, 5.0}, {{1, 3}, 4.0, 2.0}, {{1, 4}, 4.0, 2.0},
     {{2, 4}, 2.0, 5.0},
     {{3, 4}, 2.0, 1.0}}
  });

  // Inexistent source vertex.
  auto res = Dijkstra(graph, 99);
  EXPECT_TRUE(res.empty());

  // Inexistent destination vertex.
  res = Dijkstra(graph, 0, 99);
  EXPECT_TRUE(res.empty());

  // Calculate all shortest paths from 0.
  res = Dijkstra(graph, 0);

  ASSERT_NE(res.end(), res.find(0));
  EXPECT_EQ(0, res.at(0).first);
  EXPECT_EQ(0, res.at(0).second);
  ASSERT_NE(res.end(), res.find(1));
  EXPECT_EQ(3, res.at(1).first);
  EXPECT_EQ(3, res.at(1).second);
  ASSERT_NE(res.end(), res.find(2));
  EXPECT_EQ(7, res.at(2).first);
  EXPECT_EQ(4, res.at(2).second);
  ASSERT_NE(res.end(), res.find(3));
  EXPECT_EQ(1, res.at(3).first);
  EXPECT_EQ(0, res.at(3).second);
  ASSERT_NE(res.end(), res.find(4));
  EXPECT_EQ(2, res.at(4).first);
  EXPECT_EQ(3, res.at(4).second);

  // Calculate the shortest path between 0 and 1.
  res = Dijkstra(graph, 0, 1);

  ASSERT_NE(res.end(), res.find(1));
  EXPECT_EQ(3, res.at(1).first);
  EXPECT_EQ(3, res.at(1).second);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, DijkstraDirected)
{
  ///              (6)
  ///           0------>1
  ///           |      /|\
  ///           |     / | \(5)
  ///           | (2)/  |  ┘
  ///           |   /   |   2
  ///        (1)|  / (2)|  /
  ///           | /     | /(5)
  ///           VL      VL
  ///           3------>4
  ///              (1)
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"0", 0, 0}, {"1", 1, 1}, {"2", 2, 2}, {"3", 3, 3}, {"4", 4, 4}},
    // Edges.
    {{{0, 1}, 2.0, 6.0}, {{0, 3}, 3.0, 1.0},
     {{1, 2}, 4.0, 5.0}, {{1, 3}, 4.0, 2.0}, {{1, 4}, 4.0, 2.0},
     {{2, 4}, 2.0, 5.0},
     {{3, 4}, 2.0, 1.0}}
  });

  // Inexistent source vertex.
  auto res = Dijkstra(graph, 99);
  EXPECT_TRUE(res.empty());

  // Inexistent destination vertex.
  res = Dijkstra(graph, 0, 99);
  EXPECT_TRUE(res.empty());

  // Calculate all shortest paths from 0.
  res = Dijkstra(graph, 0);

  ASSERT_NE(res.end(), res.find(0));
  EXPECT_EQ(0, res.at(0).first);
  EXPECT_EQ(0, res.at(0).second);
  ASSERT_NE(res.end(), res.find(1));
  EXPECT_EQ(6, res.at(1).first);
  EXPECT_EQ(0, res.at(1).second);
  ASSERT_NE(res.end(), res.find(2));
  EXPECT_EQ(11, res.at(2).first);
  EXPECT_EQ(1, res.at(2).second);
  ASSERT_NE(res.end(), res.find(3));
  EXPECT_EQ(1, res.at(3).first);
  EXPECT_EQ(0, res.at(3).second);
  ASSERT_NE(res.end(), res.find(4));
  EXPECT_EQ(2, res.at(4).first);
  EXPECT_EQ(3, res.at(4).second);

  // Calculate the shortest path between 0 and 1.
  res = Dijkstra(graph, 0, 1);

  ASSERT_NE(res.end(), res.find(1));
  EXPECT_EQ(6, res.at(1).first);
  EXPECT_EQ(0, res.at(1).second);
}
