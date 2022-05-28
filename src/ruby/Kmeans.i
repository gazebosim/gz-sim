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

%module kmeans
%{
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Kmeans.hh>
%}

%include "std_vector.i"
%template(vector_vector3d) std::vector<gz::math::Vector3<double>>;
%template(vector_uint) std::vector<unsigned int>;

%inline %{
  struct ClusterOutput {
    bool result;
    std::vector<gz::math::Vector3<double>> centroids;
    std::vector<unsigned int> labels;
  };
%}

namespace gz
{
  namespace math
  {
    class Kmeans
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: explicit Kmeans(const std::vector<Vector3<double>> &_obs);
      public: virtual ~Kmeans();
      public: std::vector<Vector3<double>> Observations() const;
      public: bool Observations(const std::vector<Vector3<double>> &_obs);
      public: bool AppendObservations(const std::vector<Vector3<double>> &_obs);

      %pythoncode %{
      def cluster(self, _k):
        cluster_output = self._cluster(_k)
        return [cluster_output.result,
                vector_vector3d(cluster_output.centroids),
                vector_uint(cluster_output.labels)]
      %}
    };
    %extend Kmeans{
      inline ClusterOutput _cluster(int _k) {
        ClusterOutput output;
        output.result = (*$self).Cluster(_k, output.centroids, output.labels);
        return output;
      }
    }
  }
}
