/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_MOVINGWINDOWFILTER_HH_
#define IGNITION_MATH_MOVINGWINDOWFILTER_HH_

#include <memory>
#include <vector>
#include "ignition/math/Export.hh"

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //

    /// \cond
    /// \brief Private data members for MovingWindowFilter class.
    /// This must be in the header due to templatization.
    template< typename T>
    class MovingWindowFilterPrivate
    {
      // \brief Constructor
      public: MovingWindowFilterPrivate();

      /// \brief For moving window smoothed value
      public: unsigned int valWindowSize = 4;

      /// \brief buffer history of raw values
      public: std::vector<T> valHistory;

      /// \brief iterator pointing to current value in buffer
      public: typename std::vector<T>::iterator valIter;

      /// \brief keep track of running sum
      public: T sum;

      /// \brief keep track of number of elements
      public: unsigned int samples = 0;
    };
    /// \endcond

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilterPrivate<T>::MovingWindowFilterPrivate()
    {
      /// \TODO FIXME hardcoded initial value for now
      this->valHistory.resize(this->valWindowSize);
      this->valIter = this->valHistory.begin();
      this->sum = T();
    }

    /// \brief Base class for MovingWindowFilter. This replaces the
    /// version of MovingWindowFilter in the Ignition Common library.
    ///
    /// The default window size is 4.
    template< typename T>
    class MovingWindowFilter
    {
      /// \brief Constructor
      public: MovingWindowFilter();

      /// \brief Destructor
      public: virtual ~MovingWindowFilter();

      /// \brief Update value of filter
      /// \param[in] _val new raw value
      public: void Update(const T _val);

      /// \brief Set window size
      /// \param[in] _n new desired window size
      public: void SetWindowSize(const unsigned int _n);

      /// \brief Get the window size.
      /// \return The size of the moving window.
      public: unsigned int WindowSize() const;

      /// \brief Get whether the window has been filled.
      /// \return True if the window has been filled.
      public: bool WindowFilled() const;

      /// \brief Get filtered result
      /// \return Latest filtered value
      public: T Value() const;

      /// \brief Data pointer.
      private: std::unique_ptr<MovingWindowFilterPrivate<T>> dataPtr;
    };

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilter<T>::MovingWindowFilter()
    : dataPtr(new MovingWindowFilterPrivate<T>())
    {
    }

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilter<T>::~MovingWindowFilter()
    {
      this->dataPtr->valHistory.clear();
    }

    //////////////////////////////////////////////////
    template<typename T>
    void MovingWindowFilter<T>::Update(const T _val)
    {
      // update sum and sample size with incoming _val

      // keep running sum
      this->dataPtr->sum += _val;

      // shift pointer, wrap around if end has been reached.
      ++this->dataPtr->valIter;
      if (this->dataPtr->valIter == this->dataPtr->valHistory.end())
      {
        // reset iterator to beginning of queue
        this->dataPtr->valIter = this->dataPtr->valHistory.begin();
      }

      // increment sample size
      ++this->dataPtr->samples;

      if (this->dataPtr->samples > this->dataPtr->valWindowSize)
      {
        // subtract old value if buffer already filled
        this->dataPtr->sum -= (*this->dataPtr->valIter);
        // put new value into queue
        (*this->dataPtr->valIter) = _val;
        // reduce sample size
        --this->dataPtr->samples;
      }
      else
      {
        // put new value into queue
        (*this->dataPtr->valIter) = _val;
      }
    }

    //////////////////////////////////////////////////
    template<typename T>
    void MovingWindowFilter<T>::SetWindowSize(const unsigned int _n)
    {
      this->dataPtr->valWindowSize = _n;
      this->dataPtr->valHistory.clear();
      this->dataPtr->valHistory.resize(this->dataPtr->valWindowSize);
      this->dataPtr->valIter = this->dataPtr->valHistory.begin();
      this->dataPtr->sum = T();
      this->dataPtr->samples = 0;
    }

    //////////////////////////////////////////////////
    template<typename T>
    unsigned int MovingWindowFilter<T>::WindowSize() const
    {
      return this->dataPtr->valWindowSize;
    }

    //////////////////////////////////////////////////
    template<typename T>
    bool MovingWindowFilter<T>::WindowFilled() const
    {
      return this->dataPtr->samples == this->dataPtr->valWindowSize;
    }

    //////////////////////////////////////////////////
    template<typename T>
    T MovingWindowFilter<T>::Value() const
    {
      return this->dataPtr->sum / static_cast<double>(this->dataPtr->samples);
    }
    }
  }
}
#endif
