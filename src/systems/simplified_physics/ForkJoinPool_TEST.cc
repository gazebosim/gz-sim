/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "ForkJoinPool.hh"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <numeric>
#include <thread>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems::simplified_physics;

/////////////////////////////////////////////////
TEST(ForkJoinPool, ThreadCount)
{
  EXPECT_EQ(1, ForkJoinPool(0).Threads());
  EXPECT_EQ(1, ForkJoinPool(1).Threads());
  EXPECT_EQ(4, ForkJoinPool(4).Threads());
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, EveryItemRunsOnceAndWritesAreVisible)
{
  for (const int threads : {1, 2, 3, 8})
  {
    ForkJoinPool pool(threads);
    for (const std::size_t count : {0u, 1u, 2u, 7u, 100u, 1001u})
    {
      for (const std::size_t chunk : {0u, 1u, 3u, 64u, 5000u})
      {
        std::vector<int> runs(count, 0);
        std::vector<std::size_t> square(count, 0);
        pool.For(count, chunk, [&](std::size_t _i, int)
        {
          ++runs[_i];
          square[_i] = _i * _i;
        });
        for (std::size_t i = 0; i < count; ++i)
        {
          ASSERT_EQ(1, runs[i]) << threads << " threads, count " << count
              << ", chunk " << chunk << ", item " << i;
          ASSERT_EQ(i * i, square[i]);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, ThreadIndexIsInRangeAndExclusive)
{
  const int threads = 4;
  ForkJoinPool pool(threads);
  std::vector<std::atomic<int>> busy(threads);
  std::atomic<int> overlaps{0};
  std::atomic<int> outOfRange{0};
  std::vector<std::atomic<int>> perThread(threads);
  for (int round = 0; round < 200; ++round)
  {
    pool.For(256, 2, [&](std::size_t, int _thread)
    {
      if (_thread < 0 || _thread >= threads)
      {
        ++outOfRange;
        return;
      }
      if (busy[_thread].fetch_add(1) != 0)
        ++overlaps;
      ++perThread[_thread];
      busy[_thread].fetch_sub(1);
    });
  }
  EXPECT_EQ(0, outOfRange.load());
  EXPECT_EQ(0, overlaps.load());
  const int total = std::accumulate(perThread.begin(), perThread.end(), 0,
      [](int _sum, const std::atomic<int> &_n) { return _sum + _n.load(); });
  EXPECT_EQ(200 * 256, total);
  // The caller is thread 0 and always takes part.
  EXPECT_GT(perThread[0].load(), 0);
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, ManyShortLoopsInARow)
{
  // Loops that follow each other without a pause: a worker that is slow to
  // wake up must never run the body of one loop with the data of another.
  ForkJoinPool pool(8, 0);
  std::vector<std::int64_t> data(64, 0);
  for (int round = 0; round < 20000; ++round)
  {
    const std::size_t count = 2 + static_cast<std::size_t>(round % 62);
    pool.For(count, 1, [&](std::size_t _i, int)
    {
      data[_i] += round + 1;
    });
    if (round % 4999 == 0)
    {
      for (std::size_t i = 0; i < count; ++i)
        ASSERT_GE(data[i], round + 1);
    }
  }
  // Item i ran in every round whose count exceeded i.
  for (std::size_t i = 0; i < data.size(); ++i)
  {
    std::int64_t expected = 0;
    for (int round = 0; round < 20000; ++round)
    {
      if (i < 2 + static_cast<std::size_t>(round % 62))
        expected += round + 1;
    }
    EXPECT_EQ(expected, data[i]) << "item " << i;
  }
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, WorkersSleepAndWakeUp)
{
  // A zero spin time sends the workers to sleep right after a loop, so every
  // loop below has to wake them.
  ForkJoinPool pool(4, 0);
  for (int round = 0; round < 20; ++round)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    std::atomic<int> sum{0};
    pool.For(1000, 4, [&](std::size_t _i, int)
    {
      sum.fetch_add(static_cast<int>(_i));
    });
    EXPECT_EQ(999 * 1000 / 2, sum.load());
  }
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, WorkersTakeSlowItems)
{
  // The caller sleeps in its first item, so the other threads must take the
  // rest, or the loop would last 100 ms * 8.
  ForkJoinPool pool(4);
  std::atomic<int> done{0};
  const auto start = std::chrono::steady_clock::now();
  pool.For(8, 1, [&](std::size_t _i, int)
  {
    if (_i == 0)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ++done;
  });
  EXPECT_EQ(8, done.load());
  EXPECT_LT(std::chrono::steady_clock::now() - start,
      std::chrono::milliseconds(700));
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, PoolsAreIndependent)
{
  ForkJoinPool a(3);
  ForkJoinPool b(3);
  std::atomic<int> sumA{0};
  std::atomic<int> sumB{0};
  std::thread other([&]
  {
    for (int round = 0; round < 500; ++round)
      b.For(50, 1, [&](std::size_t, int) { ++sumB; });
  });
  for (int round = 0; round < 500; ++round)
    a.For(50, 1, [&](std::size_t, int) { ++sumA; });
  other.join();
  EXPECT_EQ(500 * 50, sumA.load());
  EXPECT_EQ(500 * 50, sumB.load());
}

/////////////////////////////////////////////////
TEST(ForkJoinPool, DestroyRightAfterUseAndWhileIdle)
{
  for (int round = 0; round < 50; ++round)
  {
    ForkJoinPool pool(6, round % 2 ? 0 : 200);
    if (round % 3 == 0)
    {
      std::atomic<int> n{0};
      pool.For(100, 1, [&](std::size_t, int) { ++n; });
      EXPECT_EQ(100, n.load());
    }
  }
}
