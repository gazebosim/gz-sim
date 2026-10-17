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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
constexpr int kMaxThreads{256};

/// \brief Tell the CPU that this thread is spinning.
inline void Relax()
{
#if defined(__x86_64__) || defined(__i386__)
  _mm_pause();
#elif defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#endif
}
}  // namespace

/// \brief Private data of ForkJoinPool.
///
/// Protocol. The caller (thread 0) fills the loop description and publishes it
/// by opening the job (`closed` false) and incrementing `epoch`. Workers see
/// the new epoch, register in `inside`, and check that the job is open before
/// they read the description. The caller closes the job when every iteration
/// is done and waits until no worker is inside. A worker that wakes up late
/// registers, sees a closed job and leaves, so it never reads a description
/// that the caller is rewriting. Workers never wait for each other, and the
/// caller never waits for a worker that is not inside, so a sleeping worker
/// costs nothing when the caller has done the work already.
class ForkJoinPool::Implementation
{
  /// \brief Constructor.
  public: Implementation(int _threads, int _spinMicroseconds)
      : spin(std::chrono::microseconds(std::max(0, _spinMicroseconds)))
  {
    this->threadCount = std::clamp(_threads, 1, kMaxThreads);
    this->workers.reserve(static_cast<std::size_t>(this->threadCount - 1));
    for (int id = 1; id < this->threadCount; ++id)
      this->workers.emplace_back([this, id] { this->WorkerLoop(id); });
  }

  /// \brief Stop and join the workers.
  public: ~Implementation()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->stop.store(true);
      this->epoch.fetch_add(1);
    }
    this->wake.notify_all();
    for (auto &worker : this->workers)
      worker.join();
  }

  public: Implementation(const Implementation &) = delete;
  public: Implementation &operator=(const Implementation &) = delete;

  /// \brief Take blocks of items until none is left.
  public: void Work(int _thread)
  {
    for (;;)
    {
      const std::size_t first =
          this->next.fetch_add(this->chunk, std::memory_order_relaxed);
      if (first >= this->items)
        return;
      const std::size_t last = std::min(first + this->chunk, this->items);
      for (std::size_t i = first; i < last; ++i)
        this->body(this->context, i, _thread);
      this->done.fetch_add(last - first, std::memory_order_release);
    }
  }

  public: void WorkerLoop(int _id)
  {
    std::uint64_t seen = 0;
    for (;;)
    {
      // Spin for a while, then sleep until the epoch changes.
      const auto deadline = std::chrono::steady_clock::now() + this->spin;
      int tick = 0;
      while (this->epoch.load() == seen && !this->stop.load())
      {
        Relax();
        if ((++tick & 127) == 0 &&
            std::chrono::steady_clock::now() >= deadline)
        {
          break;
        }
      }
      if (this->epoch.load() == seen && !this->stop.load())
      {
        std::unique_lock<std::mutex> lock(this->mutex);
        this->sleepers.fetch_add(1);
        this->wake.wait(lock, [&]
            { return this->epoch.load() != seen || this->stop.load(); });
        this->sleepers.fetch_sub(1);
      }
      if (this->stop.load())
        return;
      seen = this->epoch.load();

      this->inside.fetch_add(1);
      if (!this->closed.load())
        this->Work(_id);
      this->inside.fetch_sub(1);
    }
  }

  public: void Run(std::size_t _count, std::size_t _chunk, Body _body,
              void *_context)
  {
    if (_count == 0)
      return;
    _chunk = std::max<std::size_t>(_chunk, 1u);
    if (this->workers.empty() || _count <= _chunk)
    {
      for (std::size_t i = 0; i < _count; ++i)
        _body(_context, i, 0);
      return;
    }

    // No worker reads the description here: the job is closed and nobody is
    // inside.
    this->body = _body;
    this->context = _context;
    this->items = _count;
    this->chunk = _chunk;
    this->next.store(0, std::memory_order_relaxed);
    this->done.store(0, std::memory_order_relaxed);
    this->closed.store(false);
    this->epoch.fetch_add(1);
    if (this->sleepers.load() > 0)
    {
      // Taking the mutex orders this wake-up after a worker's decision to
      // sleep, so the notification cannot be lost.
      {
        std::lock_guard<std::mutex> lock(this->mutex);
      }
      this->wake.notify_all();
    }

    this->Work(0);
    while (this->done.load(std::memory_order_acquire) < _count)
      Relax();

    this->closed.store(true);
    while (this->inside.load() != 0)
      Relax();
  }

  public: int threadCount{1};
  public: std::chrono::microseconds spin;
  public: std::vector<std::thread> workers;
  public: std::mutex mutex;
  public: std::condition_variable wake;

  /// \brief Number of loops started, to tell a worker that there is news.
  public: std::atomic<std::uint64_t> epoch{0};
  public: std::atomic<int> sleepers{0};
  public: std::atomic<bool> stop{false};

  /// \brief True while the loop description may not be read.
  public: std::atomic<bool> closed{true};

  /// \brief Workers that may read the loop description.
  public: std::atomic<int> inside{0};

  // The loop description. Written by the caller while the job is closed.
  public: Body body{nullptr};
  public: void *context{nullptr};
  public: std::size_t items{0};
  public: std::size_t chunk{1};
  public: std::atomic<std::size_t> next{0};
  public: std::atomic<std::size_t> done{0};
};

//////////////////////////////////////////////////
ForkJoinPool::ForkJoinPool(int _threads, int _spinMicroseconds)
  : dataPtr(utils::MakeUniqueImpl<Implementation>(_threads, _spinMicroseconds))
{
}

//////////////////////////////////////////////////
ForkJoinPool::~ForkJoinPool() = default;

//////////////////////////////////////////////////
int ForkJoinPool::Threads() const
{
  return this->dataPtr->threadCount;
}

//////////////////////////////////////////////////
void ForkJoinPool::Run(std::size_t _count, std::size_t _chunk, Body _body,
    void *_context)
{
  this->dataPtr->Run(_count, _chunk, _body, _context);
}
}
}
}
