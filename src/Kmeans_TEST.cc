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

#include <gtest/gtest.h>
#include <vector>
#include "ignition/math/Kmeans.hh"

using namespace ignition;

//////////////////////////////////////////////////
TEST(KmeansTest, Kmeans)
{
  // Create some observations.
  std::vector<math::Vector3d> obs;
  obs.push_back(math::Vector3d(1.0, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.1, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.2, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.3, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.4, 1.0, 0.0));
  obs.push_back(math::Vector3d(5.0, 1.0, 0.0));
  obs.push_back(math::Vector3d(5.1, 1.0, 0.0));
  obs.push_back(math::Vector3d(5.2, 1.0, 0.0));
  obs.push_back(math::Vector3d(5.3, 1.0, 0.0));
  obs.push_back(math::Vector3d(5.4, 1.0, 0.0));

  // Initialize Kmeans with two partitions.
  math::Kmeans kmeans(obs);

  // ::GetObservations()
  std::vector<math::Vector3d> obsCopy;
  obsCopy = kmeans.Observations();
  for (size_t i = 0; i < obsCopy.size(); ++i)
    EXPECT_EQ(obsCopy[i], obs[i]);

  // ::SetObservations()
  for (auto &elem : obsCopy)
    elem += math::Vector3d(0.1, 0.2, 0.0);

  EXPECT_TRUE(kmeans.Observations(obsCopy));

  obsCopy = kmeans.Observations();
  for (size_t i = 0; i < obsCopy.size(); ++i)
    EXPECT_EQ(obsCopy[i], obs[i] + math::Vector3d(0.1, 0.2, 0.0));
  EXPECT_TRUE(kmeans.Observations(obs));

  // ::Cluster()
  std::vector<math::Vector3d> centroids;
  std::vector<unsigned int> labels;
  EXPECT_TRUE(kmeans.Cluster(2, centroids, labels));

  // Check that there are two centroids.
  EXPECT_EQ(centroids.size(), 2u);

  // Check that the observations are clustered properly.
  EXPECT_EQ(labels[0], labels[1]);
  EXPECT_EQ(labels[1], labels[2]);
  EXPECT_EQ(labels[2], labels[3]);
  EXPECT_EQ(labels[3], labels[4]);

  EXPECT_NE(labels[4], labels[5]);

  EXPECT_EQ(labels[5], labels[6]);
  EXPECT_EQ(labels[6], labels[7]);
  EXPECT_EQ(labels[7], labels[8]);
  EXPECT_EQ(labels[8], labels[9]);

  // Check the centroids.
  math::Vector3d expectedCentroid1(1.2, 1.0, 0.0);
  math::Vector3d expectedCentroid2(5.2, 1.0, 0.0);
  if (centroids[0] == expectedCentroid1)
    EXPECT_EQ(centroids[1], expectedCentroid2);
  else if (centroids[0] == expectedCentroid2)
    EXPECT_EQ(centroids[1], expectedCentroid1);
  else
    FAIL();

  // Try to use an empty observation vector.
  obsCopy.clear();
  EXPECT_FALSE(kmeans.Observations(obsCopy));

  // Try to call 'Cluster()' with an empty vector.
  math::Kmeans kmeansEmpty(obsCopy);
  EXPECT_FALSE(kmeansEmpty.Cluster(2, centroids, labels));

  // Try to use a non positive k.
  EXPECT_FALSE(kmeans.Cluster(0, centroids, labels));

  // Try to use a k > num_observations.
  EXPECT_FALSE(kmeans.Cluster(static_cast<int>(obs.size() + 1),
                              centroids, labels));
}

//////////////////////////////////////////////////
TEST(KmeansTest, Append)
{
  // Create some observations.
  std::vector<math::Vector3d> obs, obs2, obsTotal;

  obs.push_back(math::Vector3d(1.0, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.1, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.2, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.3, 1.0, 0.0));
  obs.push_back(math::Vector3d(1.4, 1.0, 0.0));

  obs2.push_back(math::Vector3d(5.0, 1.0, 0.0));
  obs2.push_back(math::Vector3d(5.1, 1.0, 0.0));
  obs2.push_back(math::Vector3d(5.2, 1.0, 0.0));
  obs2.push_back(math::Vector3d(5.3, 1.0, 0.0));
  obs2.push_back(math::Vector3d(5.4, 1.0, 0.0));

  obsTotal.insert(obsTotal.end(), obs.begin(), obs.end());
  obsTotal.insert(obsTotal.end(), obs2.begin(), obs2.end());

  // Initialize Kmeans with two partitions.
  math::Kmeans kmeans(obs);

  kmeans.AppendObservations(obs2);

  std::vector<math::Vector3d> obsCopy;
  obsCopy = kmeans.Observations();

  for (size_t i = 0; i < obsCopy.size(); ++i)
    EXPECT_EQ(obsTotal[i], obsCopy[i]);

  // Append an empty vector.
  std::vector<math::Vector3d> emptyVector;
  EXPECT_FALSE(kmeans.AppendObservations(emptyVector));
}
