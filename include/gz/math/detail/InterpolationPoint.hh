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

#ifndef GZ_MATH_INTERPOLATION_POINT_HH_
#define GZ_MATH_INTERPOLATION_POINT_HH_

#include <gz/math/Vector3.hh>
#include <gz/math/Vector2.hh>

#include <optional>
#include <vector>

#include <cassert>
namespace gz
{
  namespace math
  {
    /// \brief Describes an interpolation point.
    template<typename T>
    struct InterpolationPoint3D
    {
      /// \brief The position of the point
      Vector3<T> position;

      /// \brief The index from which this point was retrieved.
      /// Can be used by the application calling it to index. The reason
      /// this is optional is that data may be missing from a sparse grid.
      std::optional<std::size_t> index;
    };

    /// \brief Describes a 4D interpolation point.
    template<typename T, typename V>
    struct InterpolationPoint4D
    {
      /// \brief The relevant interpolation points in 3D
      std::vector<InterpolationPoint3D<V>> timeSlice;

      /// \brief The time of this point
      T time;
    };

    /// \brief Describes an interpolation point in1d.
    template<typename T>
    struct InterpolationPoint1D
    {
      /// \brief The position of the point
      T position;

      /// \brief The index from which this point was retrieved.
      /// Can be used by the application calling it to index.
      std::size_t index;
    };

    /// \brief Linear Interpolation of two points along a 1D line.
    /// \param[in] _a The first point.
    /// \param[in] _b The second point.
    /// \param[in] _lst An array of values that are to be used by the
    /// interpolator. _lst[a.index] and _lst[b.index] are the values
    /// to be interpolated.
    /// \param[in] _pos The position to interpolate.
    /// Warning: This function assumes that the indices of _a and _b correspond
    /// to values in _lst. It performs no bounds checking whatsoever and if you
    /// pass it invalid data, it will crash. It also assumes that _a and _b
    /// are not in the same position.
    template<typename T, typename V>
    V LinearInterpolate(
      const InterpolationPoint1D<T> &_a,
      const InterpolationPoint1D<T> &_b,
      const std::vector<V>  &_lst,
      const T &_pos
      )
    {
      assert(std::abs(_a.position - _b.position) > 0);
      assert(_a.index < _lst.size());
      assert(_b.index < _lst.size());

      auto t =  (_pos - _b.position)/ (_a.position - _b.position);
      return (1 - t) * _lst[_b.index] + t * _lst[_a.index];
    }

    /// \brief Linear Interpolation of two points in 3D space
    /// \param[in] _a The first point.
    /// \param[in] _b The second point.
    /// \param[in] _lst An array of values that are to be used by the
    /// interpolator. _lst[a.index] and _lst[b.index] are the values
    /// to be interpolated. If a.index or b.index is std::nullopt then use the
    /// default value.
    /// \param[in] _pos The position to interpolate.
    /// \param[in] _default The default value to use if a.index or b.index is
    /// std::nullopt.
    /// Warning: This function assumes that the indices of _a and _b correspond
    /// to values in _lst. It performs no bounds checking whatsoever and if you
    /// pass it invalid data, it will crash.
    template<typename T, typename V>
    V LinearInterpolate(
      const InterpolationPoint3D<T> &_a,
      const InterpolationPoint3D<T> &_b,
      const std::vector<V>  &_lst,
      const Vector3<T> &_pos,
      const V &_default = V(0)
      )
    {
      assert((_a.position - _b.position).Length() > 0);
      assert((_a.index.has_value()) ? _a.index.value() < _lst.size(): true);
      assert((_b.index.has_value()) ? _b.index.value() < _lst.size(): true);

      auto t =
        (_pos - _b.position).Length() / (_a.position - _b.position).Length();

      auto b_val = (_b.index.has_value()) ? _lst[_b.index.value()]: _default;
      auto a_val = (_a.index.has_value()) ? _lst[_a.index.value()]: _default;
      return (1 - t) * b_val + t * a_val;
    }

    /// \brief Bilinear interpolation of four points in 3D space. It assumes
    /// these 4 points form a plane.
    /// \param[in] _a The list of points to interpolate. The list must have at
    /// least 4 entries. Furthermore the order of the indices should be such
    /// that every consecutive pair of indices lie on the same edge. Furthermore
    /// the 4 points must be coplanar and corners of a rectangular patch.
    /// \param[in] _start_index The starting index of points to interpolate.
    /// \param[in] _lst An array of values that are to be used by the
    /// interpolator. _lst[a.index] and _lst[b.index] are the values
    /// to be interpolated. If a.index or b.index is std::nullopt then use the
    /// default value.
    /// \param[in] _pos The position to interpolate.
    /// \param[in] _default The default value to use if a.index or b.index is
    /// std::nullopt.
    /// Warning: This function assumes that the indices of _a and _b correspond
    /// to values in _lst. It performs no bounds checking whatsoever and if you
    /// pass it invalid data, it will crash.
    template<typename T, typename V>
    V BiLinearInterpolate(
      const std::vector<InterpolationPoint3D<T>> &_a,
      const std::size_t &_start_index,
      const std::vector<V>  &_lst,
      const Vector3<T> &_pos,
      const V &_default = V(0))
    {
      /// Assertions
      #ifndef NDEBUG
      assert(_a.size() >= _start_index + 4);
      /// Check if all points co planar.
      auto planeNormal = (_a[_start_index + 1].position -
        _a[_start_index].position).Cross(_a[_start_index + 2].position -
        _a[_start_index].position);
      auto planeScalar = planeNormal.Dot(_a[_start_index].position);
      assert(
        !(std::abs(
            planeNormal.Dot(_a[_start_index + 3].position) - planeScalar) > 0)
        );
      #endif

      // Project point onto line
      std::vector<V> linres;
      auto n0 = _a[_start_index];
      auto n1 = _a[_start_index + 1];
      auto proj1 = n1.position-n0.position;
      auto unitProj1 = proj1.Normalized();
      auto pos1 =
        (_pos - n0.position).Dot(unitProj1) * unitProj1 + n0.position;
      // Apply normal linear interpolation
      linres.push_back(LinearInterpolate(n0, n1, _lst, pos1, _default));

      // Project point onto second line
      auto n2 = _a[_start_index + 2];
      auto n3 = _a[_start_index + 3];
      auto pos2 =
        (_pos - n2.position).Dot(unitProj1) * unitProj1 + n2.position;
      linres.push_back(LinearInterpolate(n2, n3, _lst, pos2, _default));

      // Perform final linear interpolation
      InterpolationPoint3D<T> p1 {
        pos1,
        0
      };

      InterpolationPoint3D<T> p2 {
        pos2,
        1
      };
      return LinearInterpolate(p1, p2, linres, _pos, _default);
    }

    /// \brief Project Point onto a plane.
    /// \param[in] _points a vector of min length _start_index + 3. The points
    /// at _start_index, _start_index + 1 and  _start_index + 2 are used to
    /// define the plane.
    /// \param[in] _start_index defines the slice to use.
    /// \param[in] _pos The position to project onto the plane
    template<typename T>
    gz::math::Vector3<T> ProjectPointToPlane(
      const std::vector<InterpolationPoint3D<T>> _points,
      const std::size_t &_start_index,
      const Vector3<T> &_pos)
    {
      auto n =
        (_points[_start_index + 1].position - _points[_start_index].position)
        .Cross(
        _points[_start_index + 2].position - _points[_start_index].position);
      return
        _pos - n.Dot(_pos - _points[_start_index].position) * n.Normalized();
    }

    /// \brief Trilinear interpolation of eight points in 3D space. It assumes
    /// these eight points form a rectangular prism.
    /// \param[in] _a The list of points to interpolate. The list must have 8
    /// points. The first 4 points must form a plane as must the last 4.
    /// The order of the points within a plane should be such that consecutive
    /// pairs of indices lie on the same edge.
    /// \param[in] _lst An array of values that are to be used for interpolation
    /// \param[in] _pos The position to interpolate.
    /// \param[in] _default The default value to use if a.index or b.index is
    /// std::nullopt.
    template<typename T, typename V>
    V TrilinearInterpolate(
      const std::vector<InterpolationPoint3D<T>> &_a,
      const std::vector<V> &_lst,
      const Vector3<T> &_pos,
      const V &_default = V(0))
    {
      assert(_a.size() == 8);

      std::vector<V> linres;
      // First plane
      auto pos1 = ProjectPointToPlane<T>(_a, 0, _pos);
      linres.push_back(BiLinearInterpolate(_a, 0, _lst, pos1, _default));

      // Second plane
      auto pos2 = ProjectPointToPlane<T>(_a, 4, _pos);
      linres.push_back(BiLinearInterpolate(_a, 4, _lst, pos2, _default));

      // Perform final linear interpolation
      InterpolationPoint3D<T> p1 {
        pos1,
        0
      };

      InterpolationPoint3D<T> p2 {
        pos2,
        1
      };
      return LinearInterpolate(p1, p2, linres, _pos, _default);
    }
  }
}
#endif
