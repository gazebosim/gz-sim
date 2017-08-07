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
  EXPECT_EQ(res, expected);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, FindBFDirected)
{
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  // Root = 0
  for (auto dst = 0; dst < 6; ++dst)
    EXPECT_TRUE(FindBF(graph, 0, dst));

  // Root = 1
  EXPECT_FALSE(FindBF(graph, 1, 0));
  EXPECT_TRUE(FindBF(graph, 1, 1));
  EXPECT_FALSE(FindBF(graph, 1, 2));
  EXPECT_TRUE(FindBF(graph, 1, 3));
  EXPECT_TRUE(FindBF(graph, 1, 4));
  EXPECT_TRUE(FindBF(graph, 1, 5));
  EXPECT_FALSE(FindBF(graph, 1, 6));

  // Root = 2
  EXPECT_FALSE(FindBF(graph, 2, 0));
  EXPECT_FALSE(FindBF(graph, 2, 1));
  EXPECT_TRUE(FindBF(graph, 2, 2));
  EXPECT_FALSE(FindBF(graph, 2, 3));
  EXPECT_FALSE(FindBF(graph, 2, 4));
  EXPECT_FALSE(FindBF(graph, 2, 5));
  EXPECT_TRUE(FindBF(graph, 2, 6));

  // Root = 3
  EXPECT_FALSE(FindBF(graph, 3, 0));
  EXPECT_FALSE(FindBF(graph, 3, 1));
  EXPECT_FALSE(FindBF(graph, 3, 2));
  EXPECT_TRUE(FindBF(graph, 3, 3));
  EXPECT_FALSE(FindBF(graph, 3, 4));
  EXPECT_FALSE(FindBF(graph, 3, 5));
  EXPECT_FALSE(FindBF(graph, 3, 6));

  // Root = 4
  EXPECT_FALSE(FindBF(graph, 4, 0));
  EXPECT_FALSE(FindBF(graph, 4, 1));
  EXPECT_FALSE(FindBF(graph, 4, 2));
  EXPECT_FALSE(FindBF(graph, 4, 3));
  EXPECT_TRUE(FindBF(graph, 4, 4));
  EXPECT_FALSE(FindBF(graph, 4, 5));
  EXPECT_FALSE(FindBF(graph, 4, 6));

  // Root = 5
  EXPECT_FALSE(FindBF(graph, 5, 0));
  EXPECT_FALSE(FindBF(graph, 5, 1));
  EXPECT_FALSE(FindBF(graph, 5, 2));
  EXPECT_FALSE(FindBF(graph, 5, 3));
  EXPECT_TRUE(FindBF(graph, 5, 4));
  EXPECT_TRUE(FindBF(graph, 5, 5));
  EXPECT_FALSE(FindBF(graph, 54, 6));

  // Root = 6
  EXPECT_FALSE(FindBF(graph, 6, 0));
  EXPECT_FALSE(FindBF(graph, 6, 1));
  EXPECT_FALSE(FindBF(graph, 6, 2));
  EXPECT_FALSE(FindBF(graph, 6, 3));
  EXPECT_FALSE(FindBF(graph, 6, 4));
  EXPECT_FALSE(FindBF(graph, 6, 5));
  EXPECT_TRUE(FindBF(graph, 6, 6));
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, FindBFUndirected)
{
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  for (auto root = 0; root < 6; ++root)
    for (auto dst = 0; dst < 6; ++dst)
      EXPECT_TRUE(FindBF(graph, root, dst));
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
  EXPECT_EQ(res, expected);
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
  EXPECT_EQ(res, expected);
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, FindDFDirected)
{
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  // Root = 0
  for (auto dst = 0; dst < 6; ++dst)
    EXPECT_TRUE(FindDF(graph, 0, dst));

  // Root = 1
  EXPECT_FALSE(FindDF(graph, 1, 0));
  EXPECT_TRUE(FindDF(graph, 1, 1));
  EXPECT_FALSE(FindDF(graph, 1, 2));
  EXPECT_TRUE(FindDF(graph, 1, 3));
  EXPECT_TRUE(FindDF(graph, 1, 4));
  EXPECT_TRUE(FindDF(graph, 1, 5));
  EXPECT_FALSE(FindDF(graph, 1, 6));

  // Root = 2
  EXPECT_FALSE(FindDF(graph, 2, 0));
  EXPECT_FALSE(FindDF(graph, 2, 1));
  EXPECT_TRUE(FindDF(graph, 2, 2));
  EXPECT_FALSE(FindDF(graph, 2, 3));
  EXPECT_FALSE(FindDF(graph, 2, 4));
  EXPECT_FALSE(FindDF(graph, 2, 5));
  EXPECT_TRUE(FindDF(graph, 2, 6));

  // Root = 3
  EXPECT_FALSE(FindDF(graph, 3, 0));
  EXPECT_FALSE(FindDF(graph, 3, 1));
  EXPECT_FALSE(FindDF(graph, 3, 2));
  EXPECT_TRUE(FindDF(graph, 3, 3));
  EXPECT_FALSE(FindDF(graph, 3, 4));
  EXPECT_FALSE(FindDF(graph, 3, 5));
  EXPECT_FALSE(FindDF(graph, 3, 6));

  // Root = 4
  EXPECT_FALSE(FindDF(graph, 4, 0));
  EXPECT_FALSE(FindDF(graph, 4, 1));
  EXPECT_FALSE(FindDF(graph, 4, 2));
  EXPECT_FALSE(FindDF(graph, 4, 3));
  EXPECT_TRUE(FindDF(graph, 4, 4));
  EXPECT_FALSE(FindDF(graph, 4, 5));
  EXPECT_FALSE(FindDF(graph, 4, 6));

  // Root = 5
  EXPECT_FALSE(FindDF(graph, 5, 0));
  EXPECT_FALSE(FindDF(graph, 5, 1));
  EXPECT_FALSE(FindDF(graph, 5, 2));
  EXPECT_FALSE(FindDF(graph, 5, 3));
  EXPECT_TRUE(FindDF(graph, 5, 4));
  EXPECT_TRUE(FindDF(graph, 5, 5));
  EXPECT_FALSE(FindDF(graph, 54, 6));

  // Root = 6
  EXPECT_FALSE(FindDF(graph, 6, 0));
  EXPECT_FALSE(FindDF(graph, 6, 1));
  EXPECT_FALSE(FindDF(graph, 6, 2));
  EXPECT_FALSE(FindDF(graph, 6, 3));
  EXPECT_FALSE(FindDF(graph, 6, 4));
  EXPECT_FALSE(FindDF(graph, 6, 5));
  EXPECT_TRUE(FindDF(graph, 6, 6));
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, FindDFUndirected)
{
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4},
     {"F", 5, 5}, {"G", 6, 6}},
    // Edges.
    {{{0, 1}, 2.0}, {{0, 2}, 3.0}, {{0, 4}, 4.0},
     {{1, 3}, 2.0}, {{1, 5}, 3.0}, {{2, 6}, 4.0},
     {{5, 4}, 2.0}}
  });

  for (auto root = 0; root < 6; ++root)
    for (auto dst = 0; dst < 6; ++dst)
      EXPECT_TRUE(FindDF(graph, root, dst));
}

/////////////////////////////////////////////////
TEST(GraphTestFixture, DijkstraUndirected)
{
  UndirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4}},
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
  DirectedGraph<int, double> graph(
  {
    // Vertices.
    {{"A", 0, 0}, {"B", 1, 1}, {"C", 2, 2}, {"D", 3, 3}, {"E", 4, 4}},
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
