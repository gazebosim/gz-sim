/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <fstream>
#include <gz/math/Stopwatch.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
// This program will output performance comparison data between
// EntityComponentManager::Each and EntityComponentManager::EachNoCache.
// The data is stored in "each.data".
//
// Use the "each.gp" file with gnuplot to create plots of the data contained
// in "each.data". For example
//
// $ gnuplot -e "filename='each.data'" each.gp
//
// will generate a couple PNG files.
int main(int argc, char **argv)
{
  // Warm start. Initial allocation of resources can throw off calculations.
  {
    // Create the entity component manager
    EntityComponentManager mgr;

    // Create the matching entities
    for (int i = 0; i < 100; ++i)
    {
      Entity entity = mgr.CreateEntity();
      mgr.CreateComponent(entity, components::World());
      mgr.CreateComponent(entity, components::Name("world_name"));
    }

    mgr.Each<components::World, components::Name>(
        [&](const Entity &, const components::World *,
          const components::Name *)->bool {return true;});
  }

  int eachIterations = 1000;
  int maxEntityCount = 1000;
  int step = maxEntityCount / 10;

  // Open up "each.data"
  std::ofstream stream("each.data");

  // Output column headers
  stream << "# Matching_entity_count Nonmatching_entity_count "
         << "Cache Cacheless\n";

  for (int matchingEntityCount = 0; matchingEntityCount < maxEntityCount;
      matchingEntityCount += step)
  {
    for (int nonmatchingEntityCount = 0;
        nonmatchingEntityCount < maxEntityCount;
        nonmatchingEntityCount += step)
    {
      // Console output
      printf("\r%d/%d Matching %d/%d Nonmatching",
          matchingEntityCount, maxEntityCount, nonmatchingEntityCount,
          maxEntityCount);
      fflush(stdout);

      // Create the entity component manager
      EntityComponentManager mgr;
      math::Stopwatch watch;

      // Create the matching entities
      for (int i = 0; i < matchingEntityCount; ++i)
      {
        Entity worldEntity = mgr.CreateEntity();
        mgr.CreateComponent(worldEntity, components::World());
        mgr.CreateComponent(worldEntity, components::Name("world_name"));
      }

      // Create the nonmatching entities
      for (int i = 0; i < nonmatchingEntityCount; ++i)
      {
        Entity entity = mgr.CreateEntity();
        mgr.CreateComponent(entity, components::Name("world_name"));
      }

      // Calculate the duration of the cacheless version of
      // EntityComponentManager::Each
      int cachelessMatchedEntityCount = 0;
      watch.Start(true);
      for (int i = 0; i < eachIterations; ++i)
      {
        mgr.EachNoCache<components::World, components::Name>(
            [&](const Entity &, const components::World *,
              const components::Name *)->bool
            {
              cachelessMatchedEntityCount++;
              return true;
            });
      }
      watch.Stop();
      auto cachelessDuration = watch.ElapsedRunTime();

      // Calculate the duration of the cached version of
      // EntityComponentManager::Each
      int cachedMatchedEntityCount = 0;
      watch.Start(true);
      for (int i = 0; i < eachIterations; ++i)
      {
        mgr.Each<components::World, components::Name>(
            [&](const Entity &, const components::World *,
              const components::Name *)->bool
            {
              cachedMatchedEntityCount++;
              return true;
            });
      }
      watch.Stop();
      auto cacheDuration = watch.ElapsedRunTime();

      if (cachedMatchedEntityCount != matchingEntityCount * eachIterations)
      {
        std::cerr << "Chached entity count mismatch, "
          << "this should not happen!!!\n";
      }

      if (cachelessMatchedEntityCount != matchingEntityCount * eachIterations)
      {
        std::cerr << "Chacheless entity count mismatch, "
          << "this should not happen!!!\n";
      }

      double cacheIterAvg = cacheDuration.count() /
        static_cast<double>(eachIterations);
      double cacheEntityAvg = cacheIterAvg / std::max(matchingEntityCount, 1);

      double cachelessIterAvg = cachelessDuration.count() /
        static_cast<double>(eachIterations);
      double cachelessEntityAvg =
        cachelessIterAvg / std::max(matchingEntityCount, 1);

      stream << matchingEntityCount << " " << nonmatchingEntityCount << " "
        << cacheEntityAvg << " " << cachelessEntityAvg << std::endl;
    }
  }

  return 0;
}
