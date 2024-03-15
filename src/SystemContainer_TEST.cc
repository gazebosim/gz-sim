#include <gtest/gtest.h>
#include <unordered_set>
#include "SystemCointainer.hpp"

#include "iostream"
using namespace gz::sim;

//////////////////////////////////////////////////
TEST(SystemContainer, InsertionAndIteration)
{
  SystemContainer<int> cont;
  cont.Push(1);
  cont.Push(2);
  cont.Push(3);
  
  int idx = 0;
  for (auto &i: cont)
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
  for (auto &i: cont)
  {
    newCount++;
    ASSERT_NE(i, 2);
    vals.insert(i);
  }
  ASSERT_TRUE(vals.count(1) && vals.count(3));
  ASSERT_EQ(vals.size(), 2);

  std::unordered_set<int> finalVals; 
  cont.Push(2);
  for (auto &i: cont)
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
  for (auto &i: cont)
  {
    idx++;
    ASSERT_EQ(i, idx);
  }
  ASSERT_EQ(idx, 2);
}

//////////////////////////////////////////////////
TEST(SystemContainer, CheckEmptyProperties)
{
  SystemContainer<int> cont;
  ASSERT_EQ(cont.Size(), 0);
  int idx = 0;
  for (auto &i: cont)
  {
    idx++;
    ASSERT_EQ(i, idx);
  }
  ASSERT_EQ(idx, 0);
}