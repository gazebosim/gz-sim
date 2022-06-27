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

#ifndef GZ_SIM_BARRIER_HH_
#define GZ_SIM_BARRIER_HH_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    // Forward declarations.
    class BarrierPrivate;

    /// \class Barrier Barrier.hh
    /// \brief Syncronization barrier for multiple threads
    ///
    /// A Barrier is a syncronization mechanism that will block until
    /// all required threads have reached the wait() method.  This is useful
    /// for syncronizing work across many threads.
    ///
    /// Note that this can likely be replaced once the C++ concurrency TS
    /// is ratified: https://en.cppreference.com/w/cpp/experimental/barrier
    class GZ_SIM_VISIBLE Barrier
    {
      /// \brief Constructor
      /// \param[in] _threadCount Number of threads to syncronize
      /// Note: it is important to include a main thread (if used) in this
      ///       count.  For instance, controlling 10 worker threads from
      ///       1 main thread would require _threadCount=11.
      public: explicit Barrier(unsigned int _threadCount);

      /// \brief Destructor
      public: ~Barrier();

      /// \brief Enumeration of possible return values from wait()
      public: enum class ExitStatus
      {
        /// \brief Returned if the caller was not the last to call wait()
        DONE,
        /// \brief Returned if the caller was the last to call wait()
        DONE_LAST,
        /// \brief Returned if the barrier was cancelled.
        CANCELLED,
      };

      /// \brief Block until _threadCount, specified in the constructor, have
      /// reached the wait() function.
      /// \returns An exit status
      ///
      /// In general, all threads should return DONE or DONE_LAST,
      /// depending on the order that the wait function was reached.
      ///
      /// Exactly one thread in each generation of Wait will return
      /// DONE_LAST.
      ///
      /// Alternatively, if the barrier is cancelled, the ExitStatus will
      /// reflect that.
      public: ExitStatus Wait();

      /// \brief Cancel the barrier, causing all threads to unblock and
      ///        return CANCELLED
      public: void Cancel();

      /// \brief Pointer to private data.
      private: std::unique_ptr<BarrierPrivate> dataPtr;
    };
    }  // namespace GZ_SIM_VERSION_NAMESPACE
  }  // namespace sim
}  // namespace gz

#endif  // GZ_SIM_BARRIER_HH_
