/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <thread>

#include "Barrier.hh"

using namespace ignition;

//////////////////////////////////////////////////
inline bool WasCancelled(const gazebo::Barrier::ExitStatus &_ret)
{
  return _ret == gazebo::Barrier::ExitStatus::CANCELLED;
}

//////////////////////////////////////////////////
void SyncThreadsTest(unsigned int threadCount)
{
  auto barrier = std::make_unique<gazebo::Barrier>(threadCount + 1);

  unsigned int preBarrier { 0 };
  unsigned int postBarrier { 0 };
  std::vector<std::thread> threads;

  std::mutex mutex;
  std::condition_variable cv;

  for (size_t ii = 0; ii < threadCount; ++ii)
  {
    threads.push_back(std::thread([&](){
        {
          std::lock_guard<std::mutex> lock(mutex);
          preBarrier++;
          cv.notify_one();
        }

        EXPECT_EQ(barrier->Wait(),
            gazebo::Barrier::ExitStatus::DONE);

        {
          std::lock_guard<std::mutex> lock(mutex);
          postBarrier++;
          cv.notify_one();
        }
    }));
  }

  {
    std::unique_lock<std::mutex> lock(mutex);
    auto ret = cv.wait_for(lock, std::chrono::milliseconds(100),
        [&]()
        {
          return preBarrier == threadCount;
        });
    ASSERT_TRUE(ret);
  }

  // Give time for the last thread to call Wait
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(barrier->Wait(), gazebo::Barrier::ExitStatus::DONE_LAST);

  {
    std::unique_lock<std::mutex> lock(mutex);
    auto ret = cv.wait_for(lock, std::chrono::milliseconds(100),
        [&]()
        {
          return postBarrier == threadCount;
        });
    ASSERT_TRUE(ret);
  }

  for (auto & t : threads)
    t.join();
}

//////////////////////////////////////////////////
TEST(Barrier, Sync1Thread)
{
  SyncThreadsTest(1);
}

//////////////////////////////////////////////////
TEST(Barrier, Sync5Threads)
{
  SyncThreadsTest(5);
}

//////////////////////////////////////////////////
TEST(Barrier, Sync10Threads)
{
  SyncThreadsTest(10);
}

//////////////////////////////////////////////////
TEST(Barrier, Sync20Threads)
{
  SyncThreadsTest(20);
}

//////////////////////////////////////////////////
TEST(Barrier, Sync50Threads)
{
  SyncThreadsTest(50);
}

//////////////////////////////////////////////////
TEST(Barrier, Cancel)
{
  // Use 3 as number of threads, but only create one, which
  // guarantees it won't make it past `wait`
  auto barrier = std::make_unique<gazebo::Barrier>(3);

  unsigned int preBarrier { 0 };
  unsigned int postBarrier { 0 };

  std::mutex mutex;
  std::condition_variable cv;

  auto t = std::thread([&](){
      {
        std::lock_guard<std::mutex> lock(mutex);
        preBarrier++;
        cv.notify_one();
      }

      EXPECT_TRUE(WasCancelled(barrier->Wait()));

      {
        std::lock_guard<std::mutex> lock(mutex);
        postBarrier++;
        cv.notify_one();
      }
  });

  {
    std::unique_lock<std::mutex> lock(mutex);
    auto ret = cv.wait_for(lock, std::chrono::milliseconds(100),
        [&]()
        {
          return preBarrier == 1;
        });
    ASSERT_TRUE(ret);
  }

  // Cancel the barrier immedeately
  barrier->Cancel();

  {
    std::unique_lock<std::mutex> lock(mutex);
    auto ret = cv.wait_for(lock, std::chrono::milliseconds(100),
        [&]()
        {
          return postBarrier == 1;
        });
    ASSERT_TRUE(ret);
  }

  t.join();
}
