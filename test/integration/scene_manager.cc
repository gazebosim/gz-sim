/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <map>
#include <string>

#include <gz/msgs/particle_emitter.pb.h>

#include <gz/rendering/ParticleEmitter.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/rendering/SceneManager.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace gz::sim;

/// \brief Test fixture for SceneManager
class SceneManagerTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// The scatter ratio of a particle emitter message must be applied to the
// rendering particle emitter, both when it is created and when it is updated.
TEST_F(SceneManagerTest, ParticleEmitterScatterRatio)
{
  std::map<std::string, std::string> params;
  auto engine = gz::rendering::engine("ogre2", params);
  ASSERT_NE(nullptr, engine);
  auto scene = engine->CreateScene("scene_manager_test_scene");
  ASSERT_NE(nullptr, scene);

  const Entity worldId{1};
  const Entity emitterId{2};

  SceneManager sceneManager;
  sceneManager.SetScene(scene);
  sceneManager.SetWorldId(worldId);

  msgs::ParticleEmitter msg;
  msg.set_name("emitter");
  msg.mutable_particle_scatter_ratio()->set_data(0.25f);

  auto emitter = sceneManager.CreateParticleEmitter(emitterId, msg, worldId);
  ASSERT_NE(nullptr, emitter);
  EXPECT_FLOAT_EQ(0.25f, emitter->ParticleScatterRatio());

  // Update the ratio.
  msgs::ParticleEmitter update;
  update.mutable_particle_scatter_ratio()->set_data(0.75f);
  emitter = sceneManager.UpdateParticleEmitter(emitterId, update);
  ASSERT_NE(nullptr, emitter);
  EXPECT_FLOAT_EQ(0.75f, emitter->ParticleScatterRatio());

  // An update that does not contain the ratio leaves it unchanged.
  msgs::ParticleEmitter noRatio;
  noRatio.mutable_rate()->set_data(5.0);
  emitter = sceneManager.UpdateParticleEmitter(emitterId, noRatio);
  ASSERT_NE(nullptr, emitter);
  EXPECT_FLOAT_EQ(0.75f, emitter->ParticleScatterRatio());

  engine->DestroyScene(scene);
}
