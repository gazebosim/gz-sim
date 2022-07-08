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

#ifndef GZ_MATH_TIME_VARYING_VOLUMETRIC_GRID_LOOKUP_FIELD_HH_
#define GZ_MATH_TIME_VARYING_VOLUMETRIC_GRID_LOOKUP_FIELD_HH_
#include <gz/math/VolumetricGridLookupField.hh>
#include <gz/math/detail/InterpolationPoint.hh>
#include <map>
#include <optional>

namespace gz
{
  namespace math
  {
    /// \brief Lookup table for a time-varying volumetric dataset.
    /// This is an unimplimented template as the actual methods depend on the
    /// underlying structure. The key idea is that one uses a session `S` to
    /// hold a session token. This is so that we don't keep doing O(logn)
    /// lookups and instead step along the axis.
    template<typename T, typename V, typename S>
    class TimeVaryingVolumetricGridLookupField
    {
      /// \brief Constructor
      public: TimeVaryingVolumetricGridLookupField()
      {}

      /// \brief Adds a volumetric grid field.
      public: void AddVolumetricGridField(
        const T& time, const VolumetricGridLookupField<V> &_field) {}

      /// \brief Creates a session for querying
      public: S CreateSession() {}

      public: S CreateSession(const T &_time) {}

      public: std::optional<S> StepTo(const S &_session, const T &_time) {}

      public: std::pair<InterpolationPoint4D<T, V>, InterpolationPoint4D<T, V>>
      LookUp(const S &_session,
        const T &_time,
        const Vector3<V> &_point,
        const Vector3<V> &_tol) {}

      public: template<typename X>
      X EstimateQuadrilinear(
        const S &_session,
        const std::pair<InterpolationPoint4D<T, V>, InterpolationPoint4D<T, V>> &_points,
        const std::vector<X>& values1,
        const std::vector<X>& values2
      )
      {
      }
    };
    /// \brief An in-memory session. Loads the whole dataset in memory and
    /// performs queries.
    template<typename T, typename V>
    struct InMemorySession
    {
      /// \brief Iterator which holds pointer to current state
      /// TODO(arjo): Use friend to make visible only to InMemorySession
      /// specialization
      typename std::map<T, VolumetricGridLookupField<V>>::iterator iter;
    };

    /// \brief Specialized version of `TimeVaryingVolumetricGridLookupField`
    /// for in-memory lookup. It loads the whole dataset into memory.
    template<typename T, typename V>
    class TimeVaryingVolumetricGridLookupField<T, V, InMemorySession<T, V>>
    {
      public: TimeVaryingVolumetricGridLookupField()
      {}

      public: void AddVolumetricGridField(
        const T &_time, const VolumetricGridLookupField<V> &_field) {
        this->gridFields.emplace(_time, _field);
      }

      public: InMemorySession<T, V> CreateSession() {
        InMemorySession<T,V> sess;
        sess.iter = this->gridFields.begin();
        return sess;
      }

      public: InMemorySession<T, V> CreateSession(const T &_time) {
        InMemorySession<T, V> sess;
        sess.iter = this->gridFields.lower_bound(_time);
        return sess;
      }

      public: std::optional<InMemorySession<T, V>> StepTo(
        const InMemorySession<T, V> &_session, const T &_time) {
        if (_session.iter == gridFields.end())
        {
          return std::nullopt;
        }

        InMemorySession<T, V> newSess(_session);

        auto nextTime = std::next(_session.iter);
        if (nextTime == this->gridFields.end() || _time < _session.iter->first)
        {
          return std::nullopt;
        }

        while (nextTime != this->gridFields.end()
          && nextTime->first <= _time)
        {
          newSess.iter = nextTime;
          nextTime = std::next(nextTime);
        }
        return newSess;
      }

      public: std::pair<InterpolationPoint4D<T, V>, InterpolationPoint4D<T, V>>
        LookUp(const InMemorySession<T, V> &_session,
          const T &_time,
          const Vector3<V> &_point,
          const Vector3<V> &_tol = Vector3<V>{0.5, 0.5, 0.5}) {

        std::pair<InterpolationPoint4D<T, V>, InterpolationPoint4D<T, V>> res;

        if (_session.iter == this->gridFields.end())
        {

          return res;
        }

        auto nextTime = std::next(_session.iter);

        res.first.timeSlice = _session.iter->second.GetInterpolators(
            _point, _tol.X(), _tol.Y(), _tol.Z());
        res.first.time = _session.iter->first;

        if (nextTime != this->gridFields.end())
        {
          res.second.timeSlice = nextTime->second.GetInterpolators(
            _point, _tol.X(), _tol.Y(), _tol.Z());
          res.second.time = nextTime->first;
        }
        return res;
      }

      public: template<typename X>
      X EstimateQuadrilinear(
        const InMemorySession<T, V> &_session,
        const std::pair<InterpolationPoint4D<T, V>, InterpolationPoint4D<T, V>> &_interpolators,
        const std::vector<X> &_values1,
        const std::vector<X> &_values2,
        const Vector3<X> &_position,
        const T &_time,
        const X &_default = X(0)
      )
      {
        if (_interpolators.first.timeSlice.size() == 0
         && _interpolators.second.timeSlice.size() == 0)
          return _default;

        auto next = std::next(_session.iter);
        if (_interpolators.second.timeSlice.size() == 0
        || next == this->gridFields.end())
        {
          return _session.iter->second.EstimateValueUsingTrilinear(
            _position,
            _values1,
            _default
          ).value_or(_default);
        }
        if (_interpolators.first.timeSlice.size() == 0)
        {
          return next->second.EstimateValueUsingTrilinear(
            _position,
            _values2,
            _default
          ).value_or(_default);
        }

        auto res1 = _session.iter->second.EstimateValueUsingTrilinear(
          _position,
          _values1,
          _default
        );

        auto res2 = next->second.EstimateValueUsingTrilinear(
          _position,
          _values2,
          _default
        );

        InterpolationPoint1D<T>
          pt1{_session.iter->first, 0}, pt2{next->first, 1};
        std::vector<X> times{res1.value_or(_default), res2.value_or(_default)};
        return LinearInterpolate(pt1, pt2, times, _time);
      }
      private: std::map<T, VolumetricGridLookupField<V>> gridFields;
    };
  }
}
#endif
