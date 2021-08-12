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

#include <ignition/msgs/particle_emitter.pb.h>
#include <ignition/msgs/particle_emitter_v.pb.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/ParticleEmitter.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/Util.hh>
#include "ParticleEmitter2.hh"

using namespace std::chrono_literals;

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::ParticleEmitter2Private
{
  /// \brief Callback for receiving particle emitter commands.
  /// \param[in] _msg Particle emitter message.
  public: void OnCmd(const ignition::msgs::ParticleEmitter &_msg,
              const transport::MessageInfo &_info);

  public: bool EmittersService(ignition::msgs::ParticleEmitter_V &_res);

  /// \brief The transport node.
  public: ignition::transport::Node node;

  /// \brief Map of topic name to particle emitter entity.
  public: std::map<std::string, Entity> emitterTopicMap;

  /// \brief Map of Entity to particle emitter command requested externally.
  public: std::map<Entity, ignition::msgs::ParticleEmitter> userCmd;

  /// \brief A mutex to protect the user command.
  public: std::mutex mutex;

  /// \brief Protects serviceMsg.
  public: std::mutex serviceMutex;

  /// \brief Filled on demand for the emitter service.
  public: msgs::ParticleEmitter_V serviceMsg;
};

//////////////////////////////////////////////////
void ParticleEmitter2Private::OnCmd(const msgs::ParticleEmitter &_msg,
    const transport::MessageInfo &_info)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  std::map<std::string, Entity>::const_iterator iter =
    this->emitterTopicMap.find(_info.Topic());
  if (iter != this->emitterTopicMap.end())
  {
    this->userCmd[iter->second].CopyFrom(_msg);
  }
  else
  {
    ignwarn << "Topic[" << _info.Topic() << "] is not known to the particle "
      "emitter system. The requested command will be ignored.\n";
  }
}

//////////////////////////////////////////////////
bool ParticleEmitter2Private::EmittersService(
    ignition::msgs::ParticleEmitter_V &_res)
{
  _res.Clear();

  std::scoped_lock<std::mutex> lock(this->serviceMutex);
  _res.CopyFrom(this->serviceMsg);
  return true;
}

//////////////////////////////////////////////////
ParticleEmitter2::ParticleEmitter2()
  : System(), dataPtr(std::make_unique<ParticleEmitter2Private>())
{
}

//////////////////////////////////////////////////
void ParticleEmitter2::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> & /*_sdf*/,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  // World
  const components::Name *name = _ecm.Component<components::Name>(_entity);
  if (name == nullptr)
  {
    ignerr << "World with id: " << _entity
           << " has no name. ParticleEmitter2 cannot create transport topics\n";
    return;
  }

  std::string emittersService = "/world/" + name->Data() + "/particle_emitters";
  if (this->dataPtr->node.Advertise(emittersService,
        &ParticleEmitter2Private::EmittersService, this->dataPtr.get()))
  {
    ignmsg << "Serving particle emitter information on ["
      << emittersService << "]" << std::endl;
  }
  else
  {
    ignerr << "Something went wrong, failed to advertise [" << emittersService
           << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
void ParticleEmitter2::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("ParticleEmitter2::PreUpdate");

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Create particle emitters
  {
    std::lock_guard<std::mutex> serviceLock(this->dataPtr->serviceMutex);
    _ecm.EachNew<components::ParticleEmitter, components::ParentEntity,
      components::Pose>(
      [&](const Entity &_entity,
          const components::ParticleEmitter *_emitter,
          const components::ParentEntity *_parent,
          const components::Pose *_pose)->bool
        {
          std::string topic;

          // Get the topic information from the header, which is currently a
          // hack to avoid breaking the particle_emitter.proto message.
          if (_emitter->Data().has_header())
          {
            for (const auto data : _emitter->Data().header().data())
            {
              if (data.key() == "topic" && !data.value().empty())
              {
                topic = data.value(0);
              }
            }
          }

          // If a topic has not been specified, then generate topic based
          // on the scoped name.
          topic = !topic.empty() ? topic :
            topicFromScopedName(_entity, _ecm) + "/cmd";

          // Subscribe to the topic that receives particle emitter commands.
          if (!this->dataPtr->node.Subscribe(
                topic, &ParticleEmitter2Private::OnCmd, this->dataPtr.get()))
          {
            ignerr << "Error subscribing to topic [" << topic << "]. "
              << "Particle emitter will not receive updates." << std::endl;
            return false;
          }
          ignmsg << "Particle emitter["
            << scopedName(_entity, _ecm, "::", false) << "] subscribed "
            << "to command messages on topic[" << topic << "]\n";

          // Store the topic name so that we can apply user commands
          // correctly.
          this->dataPtr->emitterTopicMap[topic] = _entity;

          // Store the emitter information in the service message, which
          // can then be used in the particle_emitters service.
          msgs::ParticleEmitter *emitterMsg =
            this->dataPtr->serviceMsg.add_particle_emitter();
          emitterMsg->CopyFrom(_emitter->Data());
          msgs::Set(emitterMsg->mutable_pose(), _pose->Data());

          // Set the topic information if it was not set via SDF.
          if (!emitterMsg->has_header())
          {
            auto headerData = emitterMsg->mutable_header()->add_data();
            headerData->set_key("topic");
            headerData->add_value(topic);
          }

          // Set the particle emitter frame
          auto frameData = emitterMsg->mutable_header()->add_data();
          frameData->set_key("frame");
          frameData->add_value(
              removeParentScope(
                scopedName(_parent->Data(), _ecm, "::", false), "::"));

          return true;
        });
  }

  if (this->dataPtr->userCmd.empty() || _info.paused)
    return;

  // Process each command
  for (const auto cmd : this->dataPtr->userCmd)
  {
    // Create component.
    auto emitterComp = _ecm.Component<components::ParticleEmitterCmd>(
        cmd.first);
    if (!emitterComp)
    {
      _ecm.CreateComponent(cmd.first,
          components::ParticleEmitterCmd(cmd.second));
    }
    else
    {
      emitterComp->Data() = cmd.second;

      // Note: we process the cmd component in RenderUtil but if there is only
      // rendering on the gui side, it will not be able to remove the cmd
      // component from the ECM. It seems like adding OneTimeChange here will
      // make sure the cmd component is found again in Each call on GUI side.
      // todo(anyone) find a better way to process this cmd component in
      // RenderUtil.cc
      _ecm.SetChanged(cmd.first,
          components::ParticleEmitterCmd::typeId,
          ComponentState::OneTimeChange);
    }
  }
  this->dataPtr->userCmd.clear();
}

IGNITION_ADD_PLUGIN(ParticleEmitter2,
                    ignition::gazebo::System,
                    ParticleEmitter2::ISystemConfigure,
                    ParticleEmitter2::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ParticleEmitter2,
                          "ignition::gazebo::systems::ParticleEmitter2")
