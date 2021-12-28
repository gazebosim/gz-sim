# Copyright (C) 2021 Open Source Robotics Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
from ignition.math import Kmeans
from ignition.math import Vector3d


class TestKmeans(unittest.TestCase):

    def test_kmeans_constructor(self):
        # Create some observations.
        obs = list([])
        obs.append(Vector3d(1.0, 1.0, 0.0))
        obs.append(Vector3d(1.1, 1.0, 0.0))
        obs.append(Vector3d(1.2, 1.0, 0.0))
        obs.append(Vector3d(1.3, 1.0, 0.0))
        obs.append(Vector3d(1.4, 1.0, 0.0))
        obs.append(Vector3d(5.0, 1.0, 0.0))
        obs.append(Vector3d(5.1, 1.0, 0.0))
        obs.append(Vector3d(5.2, 1.0, 0.0))
        obs.append(Vector3d(5.3, 1.0, 0.0))
        obs.append(Vector3d(5.4, 1.0, 0.0))

        # Initialize Kmeans with two partitions.
        kmeans = Kmeans(obs)

        # ::GetObservations()
        obs_copy = list(kmeans.observations()).copy()
        for i in range(len(obs_copy)):
            self.assertEqual(obs_copy[i], obs[i])

        for idx, a in enumerate(obs_copy):
            obs_copy[idx] = a + Vector3d(0.1, 0.2, 0.0)

        self.assertTrue(kmeans.observations(obs_copy))

        obs_copy = list(kmeans.observations()).copy()
        for i in range(len(obs_copy)):
            self.assertEqual(obs_copy[i], obs[i] + Vector3d(0.1, 0.2, 0.0))
        self.assertTrue(kmeans.observations(obs))

        # ::Cluster()
        result, centroids, labels = kmeans.cluster(2)
        self.assertTrue(result)

        # Check that there are two centroids.
        self.assertEqual(len(centroids), 2)

        # Check that the observations are clustered properly.
        self.assertEqual(labels[0], labels[1])
        self.assertEqual(labels[1], labels[2])
        self.assertEqual(labels[2], labels[3])
        self.assertEqual(labels[3], labels[4])

        self.assertNotEqual(labels[4], labels[5])

        self.assertEqual(labels[5], labels[6])
        self.assertEqual(labels[6], labels[7])
        self.assertEqual(labels[7], labels[8])
        self.assertEqual(labels[8], labels[9])

        # Check the centroids.
        expected_centroid1 = Vector3d(1.2, 1.0, 0.0)
        expected_centroid2 = Vector3d(5.2, 1.0, 0.0)
        if (centroids[0] == expected_centroid1):
            self.assertEqual(centroids[1], expected_centroid2)
        elif (centroids[0] == expected_centroid2):
            self.assertEqual(centroids[1], expected_centroid1)
        else:
            self.fail()

        # Try to use an empty observation vector.
        obs_copy.clear()
        self.assertFalse(kmeans.observations(obs_copy))

        # Try to call 'Cluster()' with an empty vector.
        kmeansEmpty = Kmeans(obs_copy)
        result, centroids, labels = kmeansEmpty.cluster(2)
        self.assertFalse(result)

        # Try to use a non positive k.
        result, centroids, labels = kmeans.cluster(0)
        self.assertFalse(result)

        # Try to use a k > num_observations.
        result, centroids, labels = kmeans.cluster(len(obs) + 1)
        self.assertFalse(result)

    def test_kmeans_append(self):
        # Create some observations.
        obs = list([])
        obs2 = list([])
        obs_total = list([])

        obs.append(Vector3d(1.0, 1.0, 0.0))
        obs.append(Vector3d(1.1, 1.0, 0.0))
        obs.append(Vector3d(1.2, 1.0, 0.0))
        obs.append(Vector3d(1.3, 1.0, 0.0))
        obs.append(Vector3d(1.4, 1.0, 0.0))

        obs2.append(Vector3d(5.0, 1.0, 0.0))
        obs2.append(Vector3d(5.1, 1.0, 0.0))
        obs2.append(Vector3d(5.2, 1.0, 0.0))
        obs2.append(Vector3d(5.3, 1.0, 0.0))
        obs2.append(Vector3d(5.4, 1.0, 0.0))

        for elem in obs:
            obs_total.append(elem)

        for elem in obs2:
            obs_total.append(elem)

        # Initialize Kmeans with two partitions.
        kmeans = Kmeans(obs)

        kmeans.append_observations(obs2)

        obs_copy = kmeans.observations()

        for i in range(len(obs_copy)):
            self.assertEqual(obs_total[i], obs_copy[i])

        # Append an empty vector.
        emptyVector = []
        self.assertFalse(kmeans.append_observations(emptyVector))


if __name__ == '__main__':
    unittest.main()
