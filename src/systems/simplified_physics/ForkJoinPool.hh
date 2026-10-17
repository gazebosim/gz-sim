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
#ifndef GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_FORKJOINPOOL_HH_
#define GZ_SIM_SYSTEMS_SIMPLIFIED_PHYSICS_FORKJOINPOOL_HH_

#include <cstddef>
#include <type_traits>

#include <gz/utils/ImplPtr.hh>

#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
  /// \brief A small pool of threads that runs one loop at a time.
  ///
  /// The pool exists for loops that run every simulation step, are short
  /// (tens of microseconds to a millisecond) and whose iterations are
  /// independent. A general task queue costs more per call than these loops
  /// save, so the pool does the minimum: the calling thread publishes the
  /// loop, takes part in it, and returns when every iteration has finished.
  ///
  /// The pool owns its threads and each instance is independent, so two
  /// instances (two systems, two worlds) never share state. The threads are
  /// created once, in the constructor. After a loop they spin for a short
  /// time, which keeps the wake-up of loops that follow each other closely
  /// cheap, and then sleep, so an idle pool uses no CPU.
  ///
  /// The pool does not touch the results of the iterations. Iterations must
  /// write only to memory that no other iteration reads or writes (for
  /// example slot `i` of an output array); then the result does not depend on
  /// the number of threads or on which thread ran which iteration.
  ///
  /// The pool is not thread-safe: one thread at a time may call `For`.
  class ForkJoinPool
  {
    /// \brief Constructor.
    /// \param[in] _threads Number of threads that take part in a loop,
    /// including the caller. Values below 2 create no threads, and `For`
    /// then runs the loop on the caller. Clamped to 256.
    /// \param[in] _spinMicroseconds How long a thread that finished a loop
    /// waits for the next one before it sleeps.
    public: explicit ForkJoinPool(int _threads, int _spinMicroseconds = 50);

    /// \brief Destructor. Joins the threads. No loop may be running.
    public: ~ForkJoinPool();

    /// \brief Number of threads that take part in a loop, including the
    /// caller.
    /// \return At least 1.
    public: int Threads() const;

    /// \brief Run `_fn(item, thread)` for every `item` in `[0, _count)`.
    ///
    /// Iterations are handed out in blocks of `_chunk` consecutive items.
    /// `thread` is in `[0, Threads())`; the caller is thread 0. The calls of
    /// one thread never overlap, so a thread index can select scratch memory.
    /// The function returns after the last iteration has finished, and every
    /// write of an iteration is visible to the caller then.
    ///
    /// \param[in] _count Number of iterations.
    /// \param[in] _chunk Items per block. Zero is treated as one.
    /// \param[in] _fn A callable `void(std::size_t item, int thread)`. It
    /// must not throw and must not call `For`.
    public: template<typename Fn>
    void For(std::size_t _count, std::size_t _chunk, Fn &&_fn)
    {
      using Callable = std::remove_reference_t<Fn>;
      this->Run(_count, _chunk,
          [](void *_context, std::size_t _item, int _thread)
          {
            (*static_cast<Callable *>(_context))(_item, _thread);
          },
          const_cast<std::remove_const_t<Callable> *>(&_fn));
    }

    /// \brief Type-erased loop body.
    private: using Body = void (*)(void *, std::size_t, int);

    /// \brief Implementation of `For`.
    private: void Run(std::size_t _count, std::size_t _chunk, Body _body,
                 void *_context);

    /// \brief Private data pointer.
    GZ_UTILS_UNIQUE_IMPL_PTR(dataPtr)
  };
}
}
}

#endif
