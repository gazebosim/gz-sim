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

#ifndef IGNITION_MATH_EIGEN3_CONVERSIONS_HH_
#define IGNITION_MATH_EIGEN3_CONVERSIONS_HH_

#include <ignition/math/Matrix3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <Eigen/Geometry>

namespace ignition
{
  namespace physics
  {
    namespace dart
    {
      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      // ---------------- Converting to Eigen ----------------------
      inline Eigen::Vector3d convert(const ignition::math::Vector3d &v)
      {
        return Eigen::Vector3d(v[0], v[1], v[2]);
      }

      inline Eigen::Matrix3d convert(const ignition::math::Matrix3d &m)
      {
        Eigen::Matrix3d matrix;
        for (std::size_t i=0; i < 3; ++i)
          for (std::size_t j=0; j < 3; ++j)
            matrix(i, j) = m(i, j);

        return matrix;
      }

      inline Eigen::Quaterniond convert(const ignition::math::Quaterniond &q)
      {
        Eigen::Quaterniond quat;
        quat.w() = q.W();
        quat.x() = q.X();
        quat.y() = q.Y();
        quat.z() = q.Z();

        return quat;
      }

      inline Eigen::Isometry3d convert(const ignition::math::Pose3d &pose)
      {
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.translation() = convert(pose.Pos());
        tf.linear() = Eigen::Matrix3d(convert(pose.Rot()));

        return tf;
      }

      // vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
      // ---------------- Converting to ignition -------------------
      inline ignition::math::Vector3d convert(const Eigen::Vector3d &v)
      {
        ignition::math::Vector3d vec;
        vec.X() = v[0];
        vec.Y() = v[1];
        vec.Z() = v[2];

        return vec;
      }

      inline ignition::math::Matrix3d convert(const Eigen::Matrix3d &m)
      {
        ignition::math::Matrix3d matrix;
        for (std::size_t i=0; i < 3; ++i)
          for (std::size_t j=0; j < 3; ++j)
            matrix(i, j) = m(i, j);

        return matrix;
      }

      inline ignition::math::Quaterniond convert(const Eigen::Quaterniond &q)
      {
        ignition::math::Quaterniond quat;
        quat.W() = q.w();
        quat.X() = q.x();
        quat.Y() = q.y();
        quat.Z() = q.z();

        return quat;
      }

      inline ignition::math::Pose3d convert(const Eigen::Isometry3d &tf)
      {
        ignition::math::Pose3d pose;
        pose.Pos() = convert(Eigen::Vector3d(tf.translation()));
        pose.Rot() = convert(Eigen::Quaterniond(tf.rotation()));

        return pose;
      }
    }
  }
}

#endif
