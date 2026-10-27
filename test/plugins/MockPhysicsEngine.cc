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

// A physics engine plugin that simulates nothing. It implements the minimum
// feature list that the Physics system requires and records the sdf::World
// objects it is asked to construct, so tests can check what the Physics system
// hands over to an engine.

#include "MockPhysicsEngine.hh"

#include <string>
#include <vector>

#include <gz/physics/ForwardStep.hh>
#include <gz/physics/FrameSemantics.hh>
#include <gz/physics/FreeGroup.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Register.hh>
#include <gz/physics/RemoveEntities.hh>
#include <gz/physics/sdf/ConstructModel.hh>
#include <gz/physics/sdf/ConstructWorld.hh>

#include <sdf/Physics.hh>
#include <sdf/World.hh>

namespace gz
{
namespace sim
{
namespace test
{
namespace
{
/// \brief Same as the minimum feature list of the Physics system.
struct MockFeatureList : physics::FeatureList<
    physics::FindFreeGroupFeature,
    physics::SetFreeGroupWorldPose,
    physics::FreeGroupFrameSemantics,
    physics::LinkFrameSemantics,
    physics::ForwardStep,
    physics::RemoveModelFromWorld,
    physics::sdf::ConstructSdfModel,
    physics::sdf::ConstructSdfWorld,
    physics::GetLinkFromModel,
    physics::GetShapeFromLink>
{
};

class MockPhysicsEngine :
    public virtual physics::Implements3d<MockFeatureList>
{
  using Identity = physics::Identity;
  using FrameData = physics::FrameData<double, 3>;

  // Engine
  public: Identity InitiateEngine(std::size_t /*_engineId*/) override
  {
    return this->GenerateIdentity(0);
  }

  // ConstructSdfWorld
  public: Identity ConstructSdfWorld(const Identity &/*_engine*/,
      const ::sdf::World &_world) override
  {
    MockPhysicsEngineWorld info;
    info.name = _world.Name();
    info.gravity = _world.Gravity();
    info.physicsCount = _world.PhysicsCount();

    const auto *physics = _world.PhysicsByIndex(0);
    if (physics)
    {
      info.physicsName = physics->Name();
      info.maxStepSize = physics->MaxStepSize();
      info.realTimeFactor = physics->RealTimeFactor();

      auto elem = physics->Element();
      if (elem)
      {
        auto custom = elem->FindElement("gz:mock_engine");
        if (custom)
        {
          info.hasCustomElement = true;
          info.customElementValue = custom->Get<std::string>();
        }
      }
    }

    RecordMockPhysicsEngineWorld(info);
    return this->GenerateIdentity(MockPhysicsEngineWorlds().size());
  }

  // ForwardStep
  public: void WorldForwardStep(const Identity &/*_worldId*/,
      physics::ForwardStep::Output &/*_h*/,
      physics::ForwardStep::State &/*_x*/,
      const physics::ForwardStep::Input &/*_u*/) override
  {
  }

  // ConstructSdfModel
  public: Identity ConstructSdfModel(const Identity &/*_world*/,
      const ::sdf::Model &/*_model*/) override
  {
    return this->GenerateInvalidId();
  }

  // FrameSemantics
  public: FrameData FrameDataRelativeToWorld(
      const physics::FrameID &/*_id*/) const override
  {
    return FrameData();
  }

  // FindFreeGroupFeature
  public: Identity FindFreeGroupForModel(
      const Identity &/*_modelId*/) const override
  {
    return this->GenerateInvalidId();
  }

  public: Identity FindFreeGroupForLink(
      const Identity &/*_linkId*/) const override
  {
    return this->GenerateInvalidId();
  }

  public: Identity GetFreeGroupRootLink(
      const Identity &/*_groupId*/) const override
  {
    return this->GenerateInvalidId();
  }

  // SetFreeGroupWorldPose
  public: void SetFreeGroupWorldPose(const Identity &/*_groupId*/,
      const physics::FromPolicy<physics::FeaturePolicy3d>::Use<
          physics::Pose> &/*_pose*/) override
  {
  }

  // RemoveModelFromWorld
  public: bool RemoveModelByIndex(const Identity &/*_worldId*/,
      std::size_t /*_modelIndex*/) override
  {
    return false;
  }

  public: bool RemoveModelByName(const Identity &/*_worldId*/,
      const std::string &/*_modelName*/) override
  {
    return false;
  }

  public: bool RemoveModel(const Identity &/*_modelId*/) override
  {
    return false;
  }

  public: bool ModelRemoved(const Identity &/*_modelId*/) const override
  {
    return true;
  }

  // GetLinkFromModel
  public: std::size_t GetLinkCount(const Identity &/*_modelId*/) const override
  {
    return 0;
  }

  public: Identity GetLink(const Identity &/*_modelId*/,
      std::size_t /*_linkIndex*/) const override
  {
    return this->GenerateInvalidId();
  }

  public: Identity GetLink(const Identity &/*_modelId*/,
      const std::string &/*_linkName*/) const override
  {
    return this->GenerateInvalidId();
  }

  public: const std::string &GetLinkName(
      const Identity &/*_linkId*/) const override
  {
    static const std::string empty;
    return empty;
  }

  public: std::size_t GetLinkIndex(const Identity &/*_linkId*/) const override
  {
    return 0;
  }

  public: Identity GetModelOfLink(const Identity &/*_linkId*/) const override
  {
    return this->GenerateInvalidId();
  }

  // GetShapeFromLink
  public: std::size_t GetShapeCount(
      const Identity &/*_linkId*/) const override
  {
    return 0;
  }

  public: Identity GetShape(const Identity &/*_linkId*/,
      std::size_t /*_shapeIndex*/) const override
  {
    return this->GenerateInvalidId();
  }

  public: Identity GetShape(const Identity &/*_linkId*/,
      const std::string &/*_shapeName*/) const override
  {
    return this->GenerateInvalidId();
  }

  public: const std::string &GetShapeName(
      const Identity &/*_shapeId*/) const override
  {
    static const std::string empty;
    return empty;
  }

  public: std::size_t GetShapeIndex(
      const Identity &/*_shapeId*/) const override
  {
    return 0;
  }

  public: Identity GetLinkOfShape(const Identity &/*_shapeId*/) const override
  {
    return this->GenerateInvalidId();
  }
};

GZ_PHYSICS_ADD_PLUGIN(MockPhysicsEngine, physics::FeaturePolicy3d,
                      MockFeatureList)
}  // namespace
}  // namespace test
}  // namespace sim
}  // namespace gz
