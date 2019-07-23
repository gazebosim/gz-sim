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

inline bool WasCancelled(const gazebo::Barrier::ExitStatus &_ret)
{
  return _ret == gazebo::Barrier::ExitStatus::CANCELLED;
}

TEST(Barrier, SyncTwoThreads)
{
  auto startBarrier = std::make_unique<gazebo::Barrier>(2);
  auto stopBarrier = std::make_unique<gazebo::Barrier>(2);

  std::atomic<unsigned int> preStart { 0 };
  std::atomic<unsigned int> postStart { 0 };
  std::atomic<unsigned int> postStop { 0 };

  auto t = std::thread([&](){
      preStart++;
      EXPECT_FALSE(WasCancelled(startBarrier->wait()));
      postStart++;

      EXPECT_FALSE(WasCancelled(stopBarrier->wait()));
      postStop++;
  });


  // Sleep to let the thread start
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(preStart, 1u);

  startBarrier->wait();
  // Sleep to allow propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(postStart, 1u);

  stopBarrier->wait();
  // Sleep to allow propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(postStop, 1u);

  t.join();
}

TEST(Barrier, SyncNThreads)
{
  unsigned int threadCount = 20;

  auto startBarrier = std::make_unique<gazebo::Barrier>(threadCount + 1);
  auto stopBarrier = std::make_unique<gazebo::Barrier>(threadCount + 1);

  std::atomic<unsigned int> preStart { 0 };
  std::atomic<unsigned int> postStart { 0 };
  std::atomic<unsigned int> postStop { 0 };

  std::vector<std::thread> threads;
  for (size_t ii = 0; ii < threadCount; ++ii)
  {
    threads.push_back(std::thread([&](){
        preStart++;
        EXPECT_FALSE(WasCancelled(startBarrier->wait()));
        postStart++;

        EXPECT_FALSE(WasCancelled(stopBarrier->wait()));
        postStop++;
    }));
  }

  // Sleep to let the thread start
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(preStart, threadCount);

  startBarrier->wait();
  // Sleep to allow propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(postStart, threadCount);

  stopBarrier->wait();
  // Sleep to allow propagation
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(postStop, threadCount);

  for (auto& t : threads)
    t.join();
}

TEST(Barrier, Cancel)
{
  auto barrier = std::make_unique<gazebo::Barrier>(3);

  std::atomic<unsigned int> preBarrier { 0 };
  std::atomic<unsigned int> postBarrier { 0 };

  auto t = std::thread([&](){
      preBarrier++;
      EXPECT_TRUE(WasCancelled(barrier->wait()));
      postBarrier++;
  });

  // Cancel the barrier immedeately
  barrier->cancel();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  EXPECT_EQ(preBarrier, 1u);
  EXPECT_EQ(postBarrier, 1u);

  t.join();
}
