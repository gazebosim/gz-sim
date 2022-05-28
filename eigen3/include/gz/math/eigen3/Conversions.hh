/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GZ_MATH_EIGEN3_CONVERSIONS_HH_
#define GZ_MATH_EIGEN3_CONVERSIONS_HH_

#include <Eigen/Geometry>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace gz
{
  namespace math
  {
    namespace eigen3
    {
      /// \brief Convert from gz::math::Vector3d to Eigen::Vector3d.
      /// \param[in] _v gz::math::Vector3d to convert
      /// \return The equivalent Eigen::Vector3d.
      inline Eigen::Vector3d convert(const gz::math::Vector3d &_v)
      {
        return Eigen::Vector3d(_v[0], _v[1], _v[2]);
      }

      /// \brief Convert from gz::math::AxisAlignedBox to
      /// Eigen::AlignedBox3d.
      /// \param[in] _b gz::math::AxisAlignedBox to convert
      /// \return The equivalent Eigen::AlignedBox3d.
      inline Eigen::AlignedBox3d convert(
          const gz::math::AxisAlignedBox &_b)
      {
        return Eigen::AlignedBox3d(convert(_b.Min()), convert(_b.Max()));
      }

      /// \brief Convert from gz::math::Matrix3d to Eigen::Matrix3d.
      /// \param[in] _m gz::math::Matrix3d to convert.
      /// \return The equivalent Eigen::Matrix3d.
      inline Eigen::Matrix3d convert(const gz::math::Matrix3d &_m)
      {
        Eigen::Matrix3d matrix;
        for (std::size_t i=0; i < 3; ++i)
        {
          for (std::size_t j=0; j < 3; ++j)
          {
            matrix(i, j) = _m(i, j);
          }
        }

        return matrix;
      }

      /// \brief Convert gz::math::Quaterniond to Eigen::Quaterniond.
      /// \param[in] _q gz::math::Quaterniond to convert.
      /// \return The equivalent Eigen::Quaterniond.
      inline Eigen::Quaterniond convert(const gz::math::Quaterniond &_q)
      {
        Eigen::Quaterniond quat;
        quat.w() = _q.W();
        quat.x() = _q.X();
        quat.y() = _q.Y();
        quat.z() = _q.Z();

        return quat;
      }

      /// \brief Convert gz::math::Pose3d to Eigen::Isometry3d.
      /// \param[in] _pose gz::math::Pose3d to convert.
      /// \return The equivalent Eigen::Isometry3d.
      inline Eigen::Isometry3d convert(const gz::math::Pose3d &_pose)
      {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.translation() = convert(_pose.Pos());
        tf.linear() = Eigen::Matrix3d(convert(_pose.Rot()));

        return tf;
      }

      /// \brief Convert Eigen::Vector3d to gz::math::Vector3d.
      /// \param[in] _v Eigen::Vector3d to convert.
      /// \return The equivalent gz::math::Vector3d.
      inline gz::math::Vector3d convert(const Eigen::Vector3d &_v)
      {
        gz::math::Vector3d vec;
        vec.X() = _v[0];
        vec.Y() = _v[1];
        vec.Z() = _v[2];

        return vec;
      }

      /// \brief Convert Eigen::AlignedBox3d to gz::math::AxisAlignedBox.
      /// \param[in] _b Eigen::AlignedBox3d to convert.
      /// \return The equivalent gz::math::AxisAlignedBox.
      inline gz::math::AxisAlignedBox convert(
          const Eigen::AlignedBox3d &_b)
      {
        gz::math::AxisAlignedBox box;
        box.Min() = convert(_b.min());
        box.Max() = convert(_b.max());

        return box;
      }

      /// \brief Convert Eigen::Matrix3d to gz::math::Matrix3d.
      /// \param[in] _m Eigen::Matrix3d to convert.
      /// \return The equivalent gz::math::Matrix3d.
      inline gz::math::Matrix3d convert(const Eigen::Matrix3d &_m)
      {
        gz::math::Matrix3d matrix;
        for (std::size_t i=0; i < 3; ++i)
        {
          for (std::size_t j=0; j < 3; ++j)
          {
            matrix(i, j) = _m(i, j);
          }
        }

        return matrix;
      }

      /// \brief Convert Eigen::Quaterniond to gz::math::Quaterniond.
      /// \param[in] _q Eigen::Quaterniond to convert.
      /// \return The equivalent gz::math::Quaterniond.
      inline gz::math::Quaterniond convert(const Eigen::Quaterniond &_q)
      {
        gz::math::Quaterniond quat;
        quat.W() = _q.w();
        quat.X() = _q.x();
        quat.Y() = _q.y();
        quat.Z() = _q.z();

        return quat;
      }

      /// \brief Convert Eigen::Isometry3d to gz::math::Pose3d.
      /// \param[in] _tf Eigen::Isometry3d to convert.
      /// \return The equivalent gz::math::Pose3d.
      inline gz::math::Pose3d convert(const Eigen::Isometry3d &_tf)
      {
        gz::math::Pose3d pose;
        pose.Pos() = convert(Eigen::Vector3d(_tf.translation()));
        pose.Rot() = convert(Eigen::Quaterniond(_tf.linear()));

        return pose;
      }
    }
  }
}

#endif
