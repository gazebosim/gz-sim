/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
#include <unordered_set>
#include "SystemContainer.hpp"

using namespace gz::sim;

//////////////////////////////////////////////////
TEST(SystemContainer, InsertionAndIteration)
{
  SystemContainer<int> cont;
  cont.Push(1);
  cont.Push(2);
  cont.Push(3);

  int idx = 0;
  for (auto &i : cont)
  {
    idx++;
    ASSERT_EQ(i, idx);
  }
  ASSERT_EQ(idx, 3);
  cont.RemoveIf([](int i) {
    return i%2 == 0;
  });

  auto newCount = 0;
  std::unordered_set<int> vals;
  for (auto &i : cont)
  {
    newCount++;
    ASSERT_NE(i, 2);
    vals.insert(i);
  }
  ASSERT_TRUE(vals.count(1) && vals.count(3));
  ASSERT_EQ(vals.size(), 2);

  std::unordered_set<int> finalVals;
  cont.Push(2);
  for (auto &i : cont)
  {
    finalVals.insert(i);
  }
  ASSERT_TRUE(finalVals.count(1));
  ASSERT_TRUE(finalVals.count(2));
  ASSERT_TRUE(finalVals.count(3));
  ASSERT_EQ(finalVals.size(), 3);
}

//////////////////////////////////////////////////
TEST(SystemContainer, RemoveAtEnd)
{
  SystemContainer<int> cont;
  cont.Push(1);
  cont.Push(2);
  cont.Push(3);

  cont.RemoveIf([](int i) {
    return i == 5;
  });
  ASSERT_EQ(cont.Size(), 3);

  cont.RemoveIf([](int i) {
    return i == 3;
  });
  ASSERT_EQ(cont.Size(), 2);

  int idx = 0;
  for (auto &i : cont)
  {
    idx++;
    ASSERT_EQ(i, idx);
  }
  ASSERT_EQ(idx, 2);
}

//////////////////////////////////////////////////
TEST(SystemContainer, RemoveFromMiddle)
{
  SystemContainer<int> cont;
  for (int i = 0; i < 10; i++)
  {
    cont.Push(i);
  }

  ASSERT_EQ(cont.Size(), 10);

  cont.RemoveIf([](int i) {
    return i == 5;
  });
  ASSERT_EQ(cont.Size(), 9);


  for (auto &i : cont)
  {
    ASSERT_NE(i, 5);
  }

  cont.RemoveIf([](int i) {
    return i % 2 == 1;
  });

  for (auto &i : cont)
  {
    ASSERT_NE(i % 2, 1);
  }
  ASSERT_EQ(cont.Size(), 5);
}

//////////////////////////////////////////////////
TEST(SystemContainer, CheckEmptyProperties)
{
  SystemContainer<int> cont;
  ASSERT_EQ(cont.Size(), 0);
  int idx = 0;
  for (auto &i : cont)
  {
    idx++;
    ASSERT_EQ(i, idx);
  }
  ASSERT_EQ(idx, 0);
}
