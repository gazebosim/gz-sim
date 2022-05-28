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
#ifndef GZ_MATH_REGION3_HH_
#define GZ_MATH_REGION3_HH_

#include <cmath>
#include <limits>
#include <ostream>
#include <utility>

#include <gz/math/Interval.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/config.hh>

namespace gz
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_MATH_VERSION_NAMESPACE {
    //
    /// \class Region3 Region3.hh gz/math/Region3.hh
    /// \brief The Region3 class represents the cartesian product
    /// of intervals Ix ✕ Iy ✕ Iz, one per axis, yielding an
    /// axis-aligned region of R^3 space. It can be thought of as
    /// an intersection of halfspaces. Regions may be open or
    /// closed in their boundaries, if any.
    ///
    /// Note that the Region3 class is essentially a set R ⊆ R^3.
    /// For 3D solid box semantics, use the `AxisAlignedBox` class
    /// instead.
    ///
    /// ## Example
    ///
    /// \snippet examples/region3_example.cc complete
    template <typename T>
    class Region3
    {
      /// \brief An unbounded region (-∞, ∞) ✕ (-∞, ∞) ✕ (-∞, ∞)
      public: static const Region3<T> &Unbounded;

      /// \brief Constructor
      public: Region3() = default;

      /// \brief Constructor
      /// \param[in] _ix x-axis interval
      /// \param[in] _iy y-axis interval
      /// \param[in] _iz z-axis interval
      public: constexpr Region3(
          Interval<T> _ix, Interval<T> _iy, Interval<T> _iz)
      : ix(std::move(_ix)), iy(std::move(_iy)), iz(std::move(_iz))
      {
      }

      /// \brief Make an open region
      /// \param[in] _xLeft leftmost x-axis interval value
      /// \param[in] _xRight righmost x-axis interval value
      /// \param[in] _yLeft leftmost y-axis interval value
      /// \param[in] _yRight righmost y-axis interval value
      /// \param[in] _zLeft leftmost z-axis interval value
      /// \param[in] _zRight righmost z-axis interval value
      /// \return the (`_xLeft`, `_xRight`) ✕ (`_yLeft`, `_yRight`)
      ///  ✕ (`_zLeft`, `_zRight`) open region
      public: static constexpr Region3<T> Open(
          T _xLeft, T _yLeft, T _zLeft,
          T _xRight, T _yRight, T _zRight)
      {
        return Region3<T>(Interval<T>::Open(_xLeft, _xRight),
                          Interval<T>::Open(_yLeft, _yRight),
                          Interval<T>::Open(_zLeft, _zRight));
      }

      /// \brief Make a closed region
      /// \param[in] _xLeft leftmost x-axis interval value
      /// \param[in] _xRight righmost x-axis interval value
      /// \param[in] _yLeft leftmost y-axis interval value
      /// \param[in] _yRight righmost y-axis interval value
      /// \param[in] _zLeft leftmost z-axis interval value
      /// \param[in] _zRight righmost z-axis interval value
      /// \return the [`_xLeft`, `_xRight`] ✕ [`_yLeft`, `_yRight`]
      ///  ✕ [`_zLeft`, `_zRight`] closed region
      public: static constexpr Region3<T> Closed(
          T _xLeft, T _yLeft, T _zLeft,
          T _xRight, T _yRight, T _zRight)
      {
        return Region3<T>(Interval<T>::Closed(_xLeft, _xRight),
                          Interval<T>::Closed(_yLeft, _yRight),
                          Interval<T>::Closed(_zLeft, _zRight));
      }

      /// \brief Get the x-axis interval for the region
      /// \return the x-axis interval
      public: const Interval<T> &Ix() const { return this->ix; }

      /// \brief Get the y-axis interval for the region
      /// \return the y-axis interval
      public: const Interval<T> &Iy() const { return this->iy; }

      /// \brief Get the z-axis interval for the region
      /// \return the z-axis interval
      public: const Interval<T> &Iz() const { return this->iz; }

      /// \brief Check if the region is empty
      /// A region is empty if any of the intervals
      /// it is defined with (i.e. Ix, Iy, Iz) are.
      /// \return true if it is empty, false otherwise
      public: bool Empty() const
      {
        return this->ix.Empty() || this->iy.Empty() || this->iz.Empty();
      }

      /// \brief Check if the region contains `_point`
      /// \param[in] _point point to check for membership
      /// \return true if it is contained, false otherwise
      public: bool Contains(const Vector3<T> &_point) const
      {
        return (this->ix.Contains(_point.X()) &&
                this->iy.Contains(_point.Y()) &&
                this->iz.Contains(_point.Z()));
      }

      /// \brief Check if the region contains `_other` region
      /// \param[in] _other region to check for membership
      /// \return true if it is contained, false otherwise
      public: bool Contains(const Region3<T> &_other) const
      {
        return (this->ix.Contains(_other.ix) &&
                this->iy.Contains(_other.iy) &&
                this->iz.Contains(_other.iz));
      }

      /// \brief Check if the region intersects `_other` region
      /// \param[in] _other region to check for intersection
      /// \return true if it is contained, false otherwise
      public: bool Intersects(const Region3<T>& _other) const
      {
        return (this->ix.Intersects(_other.ix) &&
                this->iy.Intersects(_other.iy) &&
                this->iz.Intersects(_other.iz));
      }

      /// \brief Equality test operator
      /// \param _other region to check for equality
      /// \return true if regions are equal, false otherwise
      public: bool operator==(const Region3<T> &_other) const
      {
        return this->Contains(_other) && _other.Contains(*this);
      }

      /// \brief Inequality test operator
      /// \param _other region to check for inequality
      /// \return true if regions are unequal, false otherwise
      public: bool operator!=(const Region3<T> &_other) const
      {
        return !this->Contains(_other) || !_other.Contains(*this);
      }

      /// \brief Stream insertion operator
      /// \param _out output stream
      /// \param _r Region3 to output
      /// \return the stream
      public: friend std::ostream &operator<<(
        std::ostream &_out, const gz::math::Region3<T> &_r)
      {
        return _out <<_r.ix << " x " << _r.iy << " x " << _r.iz;
      }

      /// \brief The x-axis interval
      private: Interval<T> ix;
      /// \brief The y-axis interval
      private: Interval<T> iy;
      /// \brief The z-axis interval
      private: Interval<T> iz;
    };

    namespace detail {
       template<typename T>
       constexpr Region3<T> gUnboundedRegion3(
           Interval<T>::Open(-std::numeric_limits<T>::infinity(),
                             std::numeric_limits<T>::infinity()),
           Interval<T>::Open(-std::numeric_limits<T>::infinity(),
                             std::numeric_limits<T>::infinity()),
           Interval<T>::Open(-std::numeric_limits<T>::infinity(),
                             std::numeric_limits<T>::infinity()));
    }
    template<typename T>
    const Region3<T> &Region3<T>::Unbounded = detail::gUnboundedRegion3<T>;

    using Region3f = Region3<float>;
    using Region3d = Region3<double>;
    }
  }
}

#endif
