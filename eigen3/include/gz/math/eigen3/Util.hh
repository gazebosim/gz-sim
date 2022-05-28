/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GZ_MATH_EIGEN3_UTIL_HH_
#define GZ_MATH_EIGEN3_UTIL_HH_

#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/OrientedBox.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/eigen3/Conversions.hh>

namespace gz
{
  namespace math
  {
    namespace eigen3
    {
      /// \brief Get covariance matrix from a set of 3d vertices
      /// https://github.com/isl-org/Open3D/blob/76c2baf9debd460900f056a9b51e9a80de9c0e64/cpp/open3d/utility/Eigen.cpp#L305
      /// \param[in] _vertices a vector of 3d vertices
      /// \return Covariance matrix
      inline Eigen::Matrix3d covarianceMatrix(
        const std::vector<math::Vector3d> &_vertices)
      {
        if (_vertices.empty())
          return Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 9, 1> cumulants;
        cumulants.setZero();
        for (const auto &vertex : _vertices)
        {
          const Eigen::Vector3d &point = math::eigen3::convert(vertex);
          cumulants(0) += point(0);
          cumulants(1) += point(1);
          cumulants(2) += point(2);
          cumulants(3) += point(0) * point(0);
          cumulants(4) += point(0) * point(1);
          cumulants(5) += point(0) * point(2);
          cumulants(6) += point(1) * point(1);
          cumulants(7) += point(1) * point(2);
          cumulants(8) += point(2) * point(2);
        }

        Eigen::Matrix3d covariance;

        cumulants /= static_cast<double>(_vertices.size());

        covariance(0, 0) = cumulants(3) - cumulants(0) * cumulants(0);
        covariance(1, 1) = cumulants(6) - cumulants(1) * cumulants(1);
        covariance(2, 2) = cumulants(8) - cumulants(2) * cumulants(2);
        covariance(0, 1) = cumulants(4) - cumulants(0) * cumulants(1);
        covariance(1, 0) = covariance(0, 1);
        covariance(0, 2) = cumulants(5) - cumulants(0) * cumulants(2);
        covariance(2, 0) = covariance(0, 2);
        covariance(1, 2) = cumulants(7) - cumulants(1) * cumulants(2);
        covariance(2, 1) = covariance(1, 2);
        return covariance;
      }

      /// \brief Get the oriented 3d bounding box of a set of 3d
      /// vertices using PCA
      /// http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
      /// \param[in] _vertices a vector of 3d vertices
      /// \return Oriented 3D box
      inline gz::math::OrientedBoxd verticesToOrientedBox(
        const std::vector<math::Vector3d> &_vertices)
      {
        math::OrientedBoxd box;

        // Return an empty box if there are no vertices
        if (_vertices.empty())
          return box;

        math::Vector3d mean;
        for (const auto &point : _vertices)
          mean += point;
        mean /= static_cast<double>(_vertices.size());

        Eigen::Vector3d centroid = math::eigen3::convert(mean);
        Eigen::Matrix3d covariance = covarianceMatrix(_vertices);

        // Eigen Vectors
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>
          eigenSolver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3d eigenVectorsPCA = eigenSolver.eigenvectors();

        // This line is necessary for proper orientation in some cases.
        // The numbers come out the same without it, but the signs are
        // different and the box doesn't get correctly oriented in some cases.
        eigenVectorsPCA.col(2) =
          eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        // Transform the original cloud to the origin where the principal
        // components correspond to the axes.
        Eigen::Matrix4d projectionTransform(Eigen::Matrix4d::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1>(0, 3) =
          -1.0f * (projectionTransform.block<3, 3>(0, 0) * centroid);

        Eigen::Vector3d minPoint(INF_I32, INF_I32, INF_I32);
        Eigen::Vector3d maxPoint(-INF_I32, -INF_I32, -INF_I32);

        // Get the minimum and maximum points of the transformed cloud.
        for (const auto &point : _vertices)
        {
          Eigen::Vector4d pt(0, 0, 0, 1);
          pt.head<3>() = math::eigen3::convert(point);
          Eigen::Vector4d tfPoint = projectionTransform * pt;
          minPoint = minPoint.cwiseMin(tfPoint.head<3>());
          maxPoint = maxPoint.cwiseMax(tfPoint.head<3>());
        }

        const Eigen::Vector3d meanDiagonal = 0.5f * (maxPoint + minPoint);

        // quaternion is calculated using the eigenvectors (which determines
        // how the final box gets rotated), and the transform to put the box
        // in correct location is calculated
        const Eigen::Quaterniond bboxQuaternion(eigenVectorsPCA);
        const Eigen::Vector3d bboxTransform =
          eigenVectorsPCA * meanDiagonal + centroid;

        math::Vector3d size(
            maxPoint.x() - minPoint.x(),
            maxPoint.y() - minPoint.y(),
            maxPoint.z() - minPoint.z()
        );
        math::Pose3d pose;
        pose.Rot() = math::eigen3::convert(bboxQuaternion);
        pose.Pos() = math::eigen3::convert(bboxTransform);

        box.Size(size);
        box.Pose(pose);
        return box;
      }
    }
  }
}

#endif
