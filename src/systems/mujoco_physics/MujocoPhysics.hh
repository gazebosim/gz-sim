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

#ifndef GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOPHYSICS_HH_
#define GZ_SIM_SYSTEMS_MUJOCO_PHYSICS_MUJOCOPHYSICS_HH_

#include <mujoco/mujoco.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace systems
{
namespace mujoco_physics
{
class MujocoPhysics : public System,
                      public ISystemConfigure,
                      public ISystemUpdate
{
  public:
  MujocoPhysics();

  public:
  ~MujocoPhysics() override;

  public:
  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  public:
  void Update(const UpdateInfo &_info, EntityComponentManager &_ecm) override;

  private:
  void CreatePhysicsEntities(EntityComponentManager &_ecm);

  private:
  void CreateModels(EntityComponentManager &_ecm, bool &_specChanged,
                    std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody);

  private:
  void CreateCollisions(
      EntityComponentManager &_ecm, bool &_specChanged,
      const std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody);

  private:
  void CreateJoints(
      EntityComponentManager &_ecm, bool &_specChanged,
      const std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody);

  private:
  void CreateSensors(
      EntityComponentManager &_ecm, bool &_specChanged,
      const std::unordered_map<Entity, mjsBody *> &_entityToMujocoBody);

  private:
  void PostRecompileSetup(EntityComponentManager &_ecm);

  private:
  void UpdatePhysics(EntityComponentManager &_ecm);

  private:
  void ApplyPoseAndVelocityCmds(EntityComponentManager &_ecm);

  private:
  void ApplyJointAndLinkCmds(EntityComponentManager &_ecm);

  private:
  [[nodiscard]] int FindFreeJoint(Entity _canonicalLinkEntity,
                                  const EntityComponentManager &_ecm) const;

  private:
  void Step(const UpdateInfo &_info);

  private:
  void UpdateSim(const UpdateInfo &_info, EntityComponentManager &_ecm);

  private:
  void UpdatePoses(EntityComponentManager &_ecm);

  private:
  void UpdateVelocities(EntityComponentManager &_ecm);

  private:
  void UpdateSensors(EntityComponentManager &_ecm);

  private:
  void ClearResetsAndCommands(EntityComponentManager &_ecm);

  private:
  mjSpec *spec{nullptr};

  private:
  mjModel *model{nullptr};

  private:
  mjData *data{nullptr};

  private:
  std::unordered_set<Entity> worldPoseCmdsToRemove;
};
}  // namespace mujoco_physics
}  // namespace systems
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz

#endif
