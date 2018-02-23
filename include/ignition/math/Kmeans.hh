/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_KMEANS_HH_
#define IGNITION_MATH_KMEANS_HH_

#include <vector>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_MATH_VERSION_NAMESPACE {
    //
    // Forward declare private data
    class KmeansPrivate;

    /// \class Kmeans Kmeans.hh math/gzmath.hh
    /// \brief K-Means clustering algorithm. Given a set of observations,
    /// k-means partitions the observations into k sets so as to minimize the
    /// within-cluster sum of squares.
    /// Description based on http://en.wikipedia.org/wiki/K-means_clustering.
    class IGNITION_MATH_VISIBLE Kmeans
    {
      /// \brief constructor
      /// \param[in] _obs Set of observations to cluster.
      public: explicit Kmeans(const std::vector<Vector3d> &_obs);

      /// \brief Destructor.
      public: virtual ~Kmeans();

      /// \brief Get the observations to cluster.
      /// \return The vector of observations.
      public: std::vector<Vector3d> Observations() const;

      /// \brief Set the observations to cluster.
      /// \param[in] _obs The new vector of observations.
      /// \return True if the vector is not empty or false otherwise.
      public: bool Observations(const std::vector<Vector3d> &_obs);

      /// \brief Add observations to the cluster.
      /// \param[in] _obs Vector of observations.
      /// \return True if the _obs vector is not empty or false otherwise.
      public: bool AppendObservations(const std::vector<Vector3d> &_obs);

      /// \brief Executes the k-means algorithm.
      /// \param[in] _k Number of partitions to cluster.
      /// \param[out] _centroids Vector of centroids. Each element contains the
      /// centroid of one cluster.
      /// \param[out] _labels Vector of labels. The size of this vector is
      /// equals to the number of observations. Each element represents the
      /// cluster to which observation belongs.
      /// \return True when the operation succeed or false otherwise. The
      /// operation will fail if the number of observations is not positive,
      /// if the number of clusters is non positive, or if the number of
      /// clusters if greater than the number of observations.
      public: bool Cluster(int _k,
                           std::vector<Vector3d> &_centroids,
                           std::vector<unsigned int> &_labels);

      /// \brief Given an observation, it returns the closest centroid to it.
      /// \param[in] _p Point to check.
      /// \return The index of the closest centroid to the point _p.
      private: unsigned int ClosestCentroid(const Vector3d &_p) const;

      /// \brief Private data pointer
      private: KmeansPrivate *dataPtr;
    };
    }
  }
}

#endif
