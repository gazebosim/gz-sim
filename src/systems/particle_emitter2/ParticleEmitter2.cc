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
#include <ignition/gazebo/components/ParticleEmitter.hh>
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

  /// \brief Used to coordinate the emitter service response.
  public: std::condition_variable serviceCv;

  /// \brief Protects serviceMsg.
  public: std::mutex serviceMutex;

  /// \brief Filled on demand for the emitter service.
  public: msgs::ParticleEmitter_V serviceMsg;

  /// \brief True if the emitter service has been requested.
  public: bool serviceRequest = false;

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

  // Lock and wait for an iteration to be run and fill the state
  std::unique_lock<std::mutex> lock(this->serviceMutex);

  this->serviceRequest = true;
  bool success = this->serviceCv.wait_for(lock, 5s, [&]
  {
    return !this->serviceRequest;
  });

  if (success)
    _res.CopyFrom(this->serviceMsg);
  else
    ignerr << "Timed out waiting for state" << std::endl;

  return success;
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
  _ecm.EachNew<components::ParticleEmitter>(
    [&](const Entity &_entity,
        const components::ParticleEmitter *_emitter)->bool
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
        // on the model, link, and emitter names.
        if (topic.empty())
        {
          std::string emitterScopedName =
            removeParentScope(scopedName(_entity, _ecm, "/", false), "/");

          std::vector<std::string> nameParts = common::split(
              emitterScopedName, "/");

          if (nameParts.size() == 3)
          {
            topic = "/model/" + nameParts[0] + "/link/" + nameParts[1] +
              "/particle_emitter/" + nameParts[2] + "/cmd";
          }
          // Handle nested models
          else if (nameParts.size() == 4)
          {
            topic = "/model/" + nameParts[0] + "/model/" + nameParts[1] +
              "/link/" + nameParts[2] + "/particle_emitter/" + nameParts[3] +
              "/cmd";
          }
          else
          {
            ignerr << "Particle emitter missing model name, link name, or its "
            "own name.\n";
            return false;
          }
        }

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
        return true;
      });

  // Populate teh service message on demand
  if (this->dataPtr->serviceRequest)
  {
    std::unique_lock<std::mutex> lockCv(this->dataPtr->serviceMutex);
    this->dataPtr->serviceMsg.Clear();

    // Get all the particle emitters
    _ecm.Each<components::ParticleEmitter>([&](const Entity & /*_entity*/,
          const components::ParticleEmitter *_emitter)->bool
        {
          msgs::ParticleEmitter *emitterMsg =
            this->dataPtr->serviceMsg.add_particle_emitter();
          emitterMsg->CopyFrom(_emitter->Data());
          return true;
        });
    this->dataPtr->serviceRequest = false;
    this->dataPtr->serviceCv.notify_all();
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
