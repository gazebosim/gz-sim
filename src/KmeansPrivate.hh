/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_KMEANSPRIVATE_HH_
#define IGNITION_MATH_KMEANSPRIVATE_HH_

#include <vector>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/config.hh>

namespace ignition
{
  namespace math
  {
    inline namespace IGNITION_MATH_VERSION_NAMESPACE
    {
    /// \internal
    /// \brief Private data for Kmeans class
    class KmeansPrivate
    {
      /// \brief Observations.
      public: std::vector<Vector3d> obs;

      /// \brief Centroids.
      public: std::vector<Vector3d> centroids;

      /// \brief Each element stores the cluster to which observation i belongs.
      public: std::vector<unsigned int> labels;

      /// \brief Used to calculate the centroid of each partition.
      public: std::vector<Vector3d> sums;

      /// \brief Counts the number of observations contained in each partition.
      public: std::vector<unsigned int> counters;
    };
    }
  }
}
#endif


