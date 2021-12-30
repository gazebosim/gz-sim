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


from ignition.math import Kmeans, Vector3d

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

# copy original Vector to add more data
obs_copy = list(kmeans.observations()).copy()
obs2 = list([])

for idx, a in enumerate(obs_copy):
    obs_copy[idx] = a + Vector3d(0.1, 0.2, 0.0)
    obs2.append(obs_copy[idx])

# Append more observations
kmeans.append_observations(obs2)

# Calling cluster
result, centroids, labels = kmeans.cluster(2)

# Check that there are two centroids.
print("Is there a solution ? {}".format(result));
print("There are {} centroids".format(len(centroids)))
print("labels:", labels)
