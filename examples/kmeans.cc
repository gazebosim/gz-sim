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

#include <iostream>
#include <vector>

#include <gz/math/Kmeans.hh>

int main(int argc, char **argv)
{
  // Create some observations.
  std::vector<gz::math::Vector3d> obs;
  obs.push_back(gz::math::Vector3d(1.0, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(1.1, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(1.2, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(1.3, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(1.4, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(5.0, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(5.1, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(5.2, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(5.3, 1.0, 0.0));
  obs.push_back(gz::math::Vector3d(5.4, 1.0, 0.0));

  // Initialize Kmeans with two partitions.
  gz::math::Kmeans kmeans(obs);

  std::vector<gz::math::Vector3d> obsCopy;
  obsCopy = kmeans.Observations();

  for (auto &elem : obsCopy)
    elem += gz::math::Vector3d(0.1, 0.2, 0.0);

  // append more observations
  kmeans.AppendObservations(obsCopy);

  // cluster
  std::vector<gz::math::Vector3d> centroids;
  std::vector<unsigned int> labels;
  auto result = kmeans.Cluster(2, centroids, labels);

  // Check that there are two centroids.
  std::cout << "Is there a solution ? " << result << std::endl;
  std::cout << "There are " << centroids.size() << " centroids" << std::endl;
  std::cout << "labels: [";
  for (auto &label: labels)
  {
    std::cout << label << ", ";
  }
  std::cout << "]" << '\n';
}
