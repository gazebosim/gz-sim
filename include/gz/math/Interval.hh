/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef GZ_MATH_INTERVAL_HH_
#define GZ_MATH_INTERVAL_HH_

#include <cmath>
#include <limits>
#include <ostream>
#include <type_traits>
#include <utility>

#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Interval Interval.hh gz/math/Interval.hh
    /// \brief The Interval class represents a range of real numbers.
    /// Intervals may be open (a, b), left-closed [a, b), right-closed
    /// (a, b], or fully closed [a, b].
    ///
    /// ## Example
    ///
    /// \snippet examples/interval_example.cc complete
    template <typename T>
    class Interval
    {
      /// \brief An unbounded interval (-∞, ∞)
      public: static const Interval<T> &Unbounded;

      /// \brief Constructor
      public: Interval() = default;

      /// \brief Constructor
      /// \param[in] _leftValue leftmost interval value
      /// \param[in] _leftClosed whether the interval is
      ///   left-closed or not
      /// \param[in] _rightValue rightmost interval value
      /// \param[in] _rightClosed whether the interval
      ///   is right-closed or not
      public: constexpr Interval(
          T _leftValue, bool _leftClosed,
          T _rightValue, bool _rightClosed)
      : leftValue(std::move(_leftValue)),
        rightValue(std::move(_rightValue)),
        leftClosed(_leftClosed),
        rightClosed(_rightClosed)
      {
      }

      /// \brief Make an open interval (`_leftValue`, `_rightValue`)
      /// \param[in] _leftValue leftmost interval value
      /// \param[in] _rightValue rightmost interval value
      /// \return the open interval
      public: static constexpr Interval<T>
      Open(T _leftValue, T _rightValue)
      {
        return Interval<T>(
          std::move(_leftValue), false,
          std::move(_rightValue), false);
      }

      /// \brief Make a left-closed interval [`_leftValue`, `_rightValue`)
      /// \param[in] _leftValue leftmost interval value
      /// \param[in] _rightValue rightmost interval value
      /// \return the left-closed interval
      public: static constexpr Interval<T>
      LeftClosed(T _leftValue, T _rightValue)
      {
        return Interval<T>(
          std::move(_leftValue), true,
          std::move(_rightValue), false);
      }

      /// \brief Make a right-closed interval (`_leftValue`, `_rightValue`]
      /// \param[in] _leftValue leftmost interval value
      /// \param[in] _rightValue rightmost interval value
      /// \return the left-closed interval
      public: static constexpr Interval<T>
      RightClosed(T _leftValue, T _rightValue)
      {
        return Interval<T>(
          std::move(_leftValue), false,
          std::move(_rightValue), true);
      }

      /// \brief Make a closed interval [`_leftValue`, `_rightValue`]
      /// \param[in] _leftValue leftmost interval value
      /// \param[in] _rightValue rightmost interval value
      /// \return the closed interval
      public: static constexpr Interval<T>
      Closed(T _leftValue, T _rightValue)
      {
        return Interval<T>{
          std::move(_leftValue), true,
          std::move(_rightValue), true};
      }

      /// \brief Get the leftmost interval value
      /// \return the leftmost interval value
      public: const T &LeftValue() const { return this->leftValue; }

      /// \brief Check if the interval is left-closed
      /// \return true if the interval is left-closed, false otherwise
      public: bool IsLeftClosed() const { return this->leftClosed; }

      /// \brief Get the rightmost interval value
      /// \return the rightmost interval value
      public: const T &RightValue() const { return this->rightValue; }

      /// \brief Check if the interval is right-closed
      /// \return true if the interval is right-closed, false otherwise
      public: bool IsRightClosed() const { return this->rightClosed; }

      /// \brief Check if the interval is empty
      /// Some examples of empty intervals include
      /// (a, a), [a, a), and [a + 1, a].
      /// \return true if it is empty, false otherwise
      public: bool Empty() const
      {
        if (this->leftClosed && this->rightClosed)
        {
          return this->rightValue < this->leftValue;
        }
        return this->rightValue <= this->leftValue;
      }

      /// \brief Check if the interval contains `_value`
      /// \param[in] _value value to check for membership
      /// \return true if it is contained, false otherwise
      public: bool Contains(const T &_value) const
      {
        if (this->leftClosed && this->rightClosed)
        {
          return this->leftValue <= _value && _value <= this->rightValue;
        }
        if (this->leftClosed)
        {
          return this->leftValue <= _value && _value < this->rightValue;
        }
        if (this->rightClosed)
        {
          return this->leftValue < _value && _value <= this->rightValue;
        }
        return this->leftValue < _value && _value < this->rightValue;
      }

      /// \brief Check if the interval contains `_other` interval
      /// \param[in] _other interval to check for membership
      /// \return true if it is contained, false otherwise
      public: bool Contains(const Interval<T> &_other) const
      {
        if (this->Empty() || _other.Empty())
        {
          return false;
        }
        if (!this->leftClosed && _other.leftClosed)
        {
          if (_other.leftValue <= this->leftValue)
          {
            return false;
          }
        }
        else
        {
          if (_other.leftValue < this->leftValue)
          {
            return false;
          }
        }
        if (!this->rightClosed && _other.rightClosed)
        {
          if (this->rightValue <= _other.rightValue)
          {
            return false;
          }
        }
        else
        {
          if (this->rightValue < _other.rightValue)
          {
            return false;
          }
        }
        return true;
      }

      /// \brief Check if the interval intersects `_other` interval
      /// \param[in] _other interval to check for intersection
      /// \return true if both intervals intersect, false otherwise
      public: bool Intersects(const Interval<T> &_other) const
      {
        if (this->Empty() || _other.Empty())
        {
          return false;
        }
        if (this->rightClosed && _other.leftClosed)
        {
          if (this->rightValue < _other.leftValue)
          {
            return false;
          }
        }
        else
        {
          if (this->rightValue <= _other.leftValue)
          {
            return false;
          }
        }
        if (_other.rightClosed && this->leftClosed)
        {
          if (_other.rightValue < this->leftValue)
          {
            return false;
          }
        }
        else
        {
          if (_other.rightValue <= this->leftValue)
          {
            return false;
          }
        }
        return true;
      }

      /// \brief Equality test operator
      /// \param _other interval to check for equality
      /// \return true if intervals are equal, false otherwise
      public: bool operator==(const Interval<T> &_other) const
      {
        return this->Contains(_other) && _other.Contains(*this);
      }

      /// \brief Inequality test operator
      /// \param _other interval to check for inequality
      /// \return true if intervals are unequal, false otherwise
      public: bool operator!=(const Interval<T> &_other) const
      {
        return !this->Contains(_other) || !_other.Contains(*this);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _interval Interval to output
      /// \return the stream
      public: friend std::ostream &operator<<(
        std::ostream &_out, const gz::math::Interval<T> &_interval)
      {
        return _out << (_interval.leftClosed ? "[" : "(")
                    << _interval.leftValue << ", " << _interval.rightValue
                    << (_interval.rightClosed ? "]" : ")");
      }

      /// \brief The leftmost interval value
      private: T leftValue{0};
      /// \brief The righmost interval value
      private: T rightValue{0};
      /// \brief Whether the interval is left-closed or not
      private: bool leftClosed{false};
      /// \brief Whether the interval is right-closed or not
      private: bool rightClosed{false};
    };

    namespace detail {
       template<typename T>
       constexpr Interval<T> gUnboundedInterval =
           Interval<T>::Open(-std::numeric_limits<T>::infinity(),
                             std::numeric_limits<T>::infinity());
    }  // namespace detail
    template<typename T>
    const Interval<T> &Interval<T>::Unbounded = detail::gUnboundedInterval<T>;

    using Intervalf = Interval<float>;
    using Intervald = Interval<double>;
    }
  }
}

#endif
