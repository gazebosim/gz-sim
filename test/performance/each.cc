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


#include <gtest/gtest.h>
#include <gz/math/Stopwatch.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/components/Name.hh"

using namespace gz;
using namespace sim;

void warmstart()
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

TEST(EntityComponentManagerPerfrormance, Each)
{
  int eachIterations = 100;
  int maxEntityCount = 1000;
  int step = maxEntityCount/10;

  // Initial allocation of resources can throw off calculations.
  warmstart();

  for (int matchingEntityCount = 10; matchingEntityCount < maxEntityCount;
       matchingEntityCount += step)
  {
    for (int nonmatchingEntityCount = 10;
         nonmatchingEntityCount < maxEntityCount;
         nonmatchingEntityCount += step)
    {
      EntityComponentManager mgr;

      for (int i = 0; i < matchingEntityCount; ++i)
      {
        Entity worldEntity = mgr.CreateEntity();
        mgr.CreateComponent(worldEntity, components::World());
        mgr.CreateComponent(worldEntity, components::Name("world_name"));
      }

      for (int i = 0; i < nonmatchingEntityCount; ++i)
      {
        Entity entity = mgr.CreateEntity();
        mgr.CreateComponent(entity, components::Name("world_name"));
      }

      // Calculate the duration of the cached version of
      // EntityComponentManager::Each
      math::Stopwatch watch;
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

      // Make sure the entity count matches between cached and not cached
      EXPECT_EQ(cachedMatchedEntityCount,
          matchingEntityCount * eachIterations);
      EXPECT_EQ(cachelessMatchedEntityCount,
          matchingEntityCount * eachIterations);

      double cacheIterAvg = cacheDuration.count() /
        static_cast<double>(eachIterations);
      double cacheEntityAvg = cacheIterAvg / matchingEntityCount;

      double cachelessIterAvg = cachelessDuration.count() /
        static_cast<double>(eachIterations);
      double cachelessEntityAvg = cachelessIterAvg / matchingEntityCount;

      EXPECT_LT(cacheEntityAvg, cachelessEntityAvg)
        << "Matching Entity Count =\t\t"
        << matchingEntityCount << "\n"
        << "Nonmatching Entity Count =\t" << nonmatchingEntityCount << "\n"
        << "Each Iterations =\t\t" << eachIterations << "\n"
        << "Cache total =\t\t\t" << cacheDuration.count() << " ns\n"
        << "Cache avg per iter =\t\t" << cacheIterAvg << " ns\n"
        << "Cache avg per iter*entity =\t" << cacheEntityAvg << " ns\n"
        << "Cacheless total =\t\t" << cachelessDuration.count() << " ns\n"
        << "Cacheless avg per iter=\t\t" << cachelessIterAvg << " ns\n"
        << "Cacheless avg per iter*entity=\t" << cachelessEntityAvg << " ns\n";
    }
  }
}
