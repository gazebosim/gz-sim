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

#include <iostream>
#include <ignition/math/Kmeans.hh>
#include <ignition/math/Rand.hh>
#include "KmeansPrivate.hh"

using namespace ignition;
using namespace math;

//////////////////////////////////////////////////
Kmeans::Kmeans(const std::vector<Vector3d> &_obs)
: dataPtr(new KmeansPrivate)
{
  this->Observations(_obs);
}

//////////////////////////////////////////////////
Kmeans::~Kmeans()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

//////////////////////////////////////////////////
std::vector<Vector3d> Kmeans::Observations() const
{
  return this->dataPtr->obs;
}

//////////////////////////////////////////////////
bool Kmeans::Observations(const std::vector<Vector3d> &_obs)
{
  if (_obs.empty())
  {
    std::cerr << "Kmeans::SetObservations() error: Observations vector is empty"
              << std::endl;
    return false;
  }
  this->dataPtr->obs = _obs;
  return true;
}

//////////////////////////////////////////////////
bool Kmeans::AppendObservations(const std::vector<Vector3d> &_obs)
{
  if (_obs.empty())
  {
    std::cerr << "Kmeans::AppendObservations() error: input vector is empty"
              << std::endl;
    return false;
  }
  this->dataPtr->obs.insert(this->dataPtr->obs.end(), _obs.begin(), _obs.end());
  return true;
}

//////////////////////////////////////////////////
bool Kmeans::Cluster(int _k,
                     std::vector<Vector3d> &_centroids,
                     std::vector<unsigned int> &_labels)
{
  // Sanity check.
  if (this->dataPtr->obs.empty())
  {
    std::cerr << "Kmeans error: The set of observations is empty" << std::endl;
    return false;
  }

  if (_k <= 0)
  {
    std::cerr << "Kmeans error: The number of clusters has to"
              << " be positive but its value is [" << _k << "]"
              << std::endl;
    return false;
  }

  if (_k > static_cast<int>(this->dataPtr->obs.size()))
  {
    std::cerr << "Kmeans error: The number of clusters [" << _k << "] has to be"
              << " lower or equal to the number of observations ["
              << this->dataPtr->obs.size() << "]" << std::endl;
    return false;
  }

  size_t changed = 0;

  // Initialize the size of the vectors;
  this->dataPtr->centroids.clear();
  this->dataPtr->labels.resize(this->dataPtr->obs.size());
  this->dataPtr->sums.resize(_k);
  this->dataPtr->counters.resize(_k);

  for (auto i = 0; i < _k; ++i)
  {
    // Choose a random observation and make sure it has not been chosen before.
    // Note: This is not really random but it's faster than choosing a random
    // one and verifying that it was not taken before.
    this->dataPtr->centroids.push_back(this->dataPtr->obs[i]);
  }

  // Initialize labels.
  for (auto i = 0u; i < this->dataPtr->obs.size(); ++i)
    this->dataPtr->labels[i] = 0;

  do
  {
    // Reset sums and counters.
    for (auto i = 0u; i < this->dataPtr->centroids.size(); ++i)
    {
      this->dataPtr->sums[i] = Vector3d::Zero;
      this->dataPtr->counters[i] = 0;
    }
    changed = 0;

    for (auto i = 0u; i < this->dataPtr->obs.size(); ++i)
    {
      // Update the labels containing the closest centroid for each point.
      auto label = this->ClosestCentroid(this->dataPtr->obs[i]);
      if (this->dataPtr->labels[i] != label)
      {
        this->dataPtr->labels[i] = label;
        changed++;
      }
      this->dataPtr->sums[label] += this->dataPtr->obs[i];
      this->dataPtr->counters[label]++;
    }

    // Update the centroids.
    for (auto i = 0u; i < this->dataPtr->centroids.size(); ++i)
    {
      this->dataPtr->centroids[i] =
        this->dataPtr->sums[i] / this->dataPtr->counters[i];
    }
  }
  while (changed > (this->dataPtr->obs.size() >> 10));

  _centroids = this->dataPtr->centroids;
  _labels = this->dataPtr->labels;
  return true;
}

//////////////////////////////////////////////////
unsigned int Kmeans::ClosestCentroid(const Vector3d &_p) const
{
  double min = HUGE_VAL;
  unsigned int minIdx = 0;
  for (auto i = 0u; i < this->dataPtr->centroids.size(); ++i)
  {
    double d = _p.Distance(this->dataPtr->centroids[i]);
    if (d < min)
    {
      min = d;
      minIdx = i;
    }
  }
  return minIdx;
}
