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
#include <string>
#include <vector>

#include "Kmeans.hh"
#include <gz/math/Kmeans.hh>
#include <pybind11/stl.h>

namespace gz
{
namespace math
{
namespace python
{
void defineMathKmeans(py::module &m, const std::string &typestr)
{
  using Class = gz::math::Kmeans;
  std::string pyclass_name = typestr;
  py::class_<Class>(m,
                    pyclass_name.c_str(),
                    py::buffer_protocol(),
                    py::dynamic_attr())
  .def(py::init<const std::vector<gz::math::Vector3d>&>())
  .def("observations",
       py::overload_cast<const std::vector<gz::math::Vector3d>&>
        (&Class::Observations),
        "Set the observations to cluster.")
  .def("observations",
       py::overload_cast<>(&Class::Observations, py::const_),
       "Get the observations to cluster.")
  .def("append_observations",
       &Class::AppendObservations,
       "Add observations to the cluster.")
  .def("cluster",
       [](Class &self, int k) {
         std::vector<gz::math::Vector3<double>> centroids;
         std::vector<unsigned int> labels;
         bool result = self.Cluster(k, centroids, labels);
         return std::make_tuple(result, centroids, labels);
       },
       "Executes the k-means algorithm.");
}
}  // namespace python
}  // namespace math
}  // namespace gz
