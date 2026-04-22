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

#include "Barrier.hh"

class gz::sim::BarrierPrivate
{
  /// \brief Mutex for synchronization
  public: std::mutex mutex;

  /// \brief Condition Variable for signaling
  public: std::condition_variable cv;

  /// \brief Flag to indicate if the barrier was cancelled
  public: std::atomic<bool> cancelled { false };

  /// \brief Number of participating threads
  public: unsigned int threadCount;

  /// \brief Current remaining thread count (decrements from threadCount)
  public: std::atomic<unsigned int> count;

  /// \brief Barrier generation, incremented when all threads report
  public: std::atomic<unsigned int> generation{0};
};

using namespace gz::sim;

//////////////////////////////////////////////////
Barrier::Barrier(unsigned int _threadCount)
  : dataPtr(std::make_unique<BarrierPrivate>())
{
  this->dataPtr->threadCount = _threadCount;
  this->dataPtr->count = _threadCount;
}

//////////////////////////////////////////////////
Barrier::~Barrier() = default;

//////////////////////////////////////////////////
Barrier::ExitStatus Barrier::Wait()
{
  if (this->dataPtr->cancelled)
  {
    return Barrier::ExitStatus::CANCELLED;
  }

  unsigned int currentGeneration = this->dataPtr->generation.load(std::memory_order_acquire);

  if (this->dataPtr->count.fetch_sub(1, std::memory_order_acq_rel) == 1)
  {
    this->dataPtr->generation.fetch_add(1, std::memory_order_release);
    this->dataPtr->count.store(this->dataPtr->threadCount, std::memory_order_release);
    this->dataPtr->cv.notify_all();
    return Barrier::ExitStatus::DONE_LAST;
  }

  unsigned int spinCount = 1000;
  while (this->dataPtr->generation.load(std::memory_order_acquire) == currentGeneration)
  {
    if (this->dataPtr->cancelled)
    {
      return Barrier::ExitStatus::CANCELLED;
    }

    if (--spinCount == 0)
    {
      std::unique_lock<std::mutex> lock(this->dataPtr->mutex);
      if (this->dataPtr->generation.load(std::memory_order_acquire) != currentGeneration)
        break;

      this->dataPtr->cv.wait(lock);
      break;
    }

#if defined(__x86_64__)
    __builtin_ia32_pause();
#elif defined(__aarch64__)
    asm volatile("yield");
#else
    std::this_thread::yield();
#endif
  }

  if (this->dataPtr->cancelled)
  {
    return Barrier::ExitStatus::CANCELLED;
  }
  
  return Barrier::ExitStatus::DONE;
}

//////////////////////////////////////////////////
void Barrier::Cancel()
{
  std::unique_lock<std::mutex> lock(this->dataPtr->mutex);
  // This forces pending threads to release
  this->dataPtr->generation++;
  this->dataPtr->cancelled = true;
  this->dataPtr->cv.notify_all();
}
