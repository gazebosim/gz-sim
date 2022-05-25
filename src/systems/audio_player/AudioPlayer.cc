/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <ignition/msgs/param.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <chrono>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/AngularVelocityCmd.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"


#include "AudioPlayer.hh"
#include "OpenAL.hh"

using namespace std::chrono_literals;
using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private dat for OpenAL
class ignition::gazebo::systems::AudioPlayer::Implementation
{
  /// \brief The link entity.
  public: ignition::gazebo::Link link;

  /// \brief Model interface.
  public: Model model{kNullEntity};

  /// \brief OpenAL.
  public: OpenAL openal;

  /// \brief All the audio sources.
  public: std::vector<OpenALSourcePtr> audioSources;

  /// \brief An audio sink.
  public: OpenALSinkPtr audioSink;

  /// \brief A Param message containing an audio request.
  public: ignition::msgs::Param paramMsg;

  /// \brief An Ignition Transport node.
  public: ignition::transport::Node node;

  /// \brief A publisher to send audio control messages.
  public: ignition::transport::Node::Publisher audioPub;

  /// \brief Topic to publish audio control commands.
  public: std::string audioControlTopic = "audio/control";

  /// \brief Last simulation time we publish an audio command.
  public: std::chrono::steady_clock::duration lastCommandTime{-1};

  /// \brief Time between audio commands.
  public: std::chrono::steady_clock::duration audioPeriod{0};

  /// \brief Play an audio file?
  public: bool playback = true;
};

//////////////////////////////////////////////////
AudioPlayer::AudioPlayer()
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  ignerr << "AudioPlayer loaded" << std::endl;
}

//////////////////////////////////////////////////
void AudioPlayer::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
//  #ifdef HAVE_OPENAL
  if (_sdf->HasElement("audio"))
  {
    sdf::ElementPtr audioElem = _sdf->Clone()->GetElement("audio");
    this->dataPtr->openal.Load(audioElem);
    ignerr << "Devices: " << std::endl;
    for (auto device : this->dataPtr->openal.DeviceList())
    {
      ignerr << device << std::endl;
    }
  }

  if (_sdf->HasElement("audio_source"))
  {
    sdf::ElementPtr audioElem = _sdf->Clone()->GetElement("audio_source");

    if (!audioElem->HasElement("link_name"))
    {
      ignerr << "No <link_name> specified" << std::endl;
      return;
    }

    this->dataPtr->model = Model(_entity);
    std::string linkName = audioElem->Get<std::string>("link_name");
    this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
    if (!this->dataPtr->link.Valid(_ecm))
    {
      ignerr << "Could not find link named [" << linkName
             << "] in model" << std::endl;
      return;
    }

    // bool onContact = false;
    std::vector<std::string> collisionNames;

    while (audioElem)
    {
      OpenALSourcePtr source = this->dataPtr->openal.CreateSource(audioElem);

      std::vector<std::string> names = source->CollisionNames();
      std::copy(names.begin(), names.end(), std::back_inserter(collisionNames));

      audioElem = audioElem->GetNextElement("audio_source");
      this->dataPtr->audioSources.push_back(source);
    }

    // if (!collisionNames.empty())
    // {
    //   for (std::vector<std::string>::iterator iter = collisionNames.begin();
    //       iter != collisionNames.end(); ++iter)
    //   {
    //     (*iter) = this->GetScopedName() + "::" + (*iter);
    //   }

    //   std::string topic =
    //     this->world->Physics()->GetContactManager()->CreateFilter(
    //         this->GetScopedName() + "/audio_collision", collisionNames);
    //   this->dataPtr->audioContactsSub = this->node->Subscribe(topic,
    //       &Link::OnCollision, this);
    // }
  }

  if (_sdf->HasElement("audio_sink"))
  {
    sdf::ElementPtr audioSinkElem = _sdf->Clone()->GetElement("audio_sink");

    if (!audioSinkElem->HasElement("link_name"))
    {
      ignerr << "No <link_name> specified" << std::endl;
      return;
    }

    this->dataPtr->model = Model(_entity);
    std::string linkName = audioSinkElem->Get<std::string>("link_name");
    this->dataPtr->link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
    if (!this->dataPtr->link.Valid(_ecm))
    {
      ignerr << "Could not find link named [" << linkName
             << "] in model" << std::endl;
      return;
    }

    this->dataPtr->audioSink = this->dataPtr->openal.CreateSink(audioSinkElem);
  }

  this->dataPtr->audioPub =
    this->dataPtr->node.Advertise<ignition::msgs::Param>(
      this->dataPtr->audioControlTopic);
  if (!this->dataPtr->audioPub)
  {
    std::cerr << "Error advertising topic ["
              << this->dataPtr->audioControlTopic << "]" << std::endl;
    return;
  }
}

//////////////////////////////////////////////////
void AudioPlayer::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("AudioPlayer::PreUpdate");

  if (_info.paused)
    return;

  double rate = 1;
  std::chrono::duration<double> period{rate > 0 ? 1 / rate : 0};
  this->dataPtr->audioPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

  // this->dataPtr->audioSink->SetPose(ignition::math::Pose3d::Zero);
  // this->dataPtr->audioSink->SetVelocity(ignition::math::Vector3d::Zero);

  // for (auto &audioSource : this->dataPtr->audioSources)
  // {
  //   audioSource->SetPose({1, 1, 0, 0, 0, 0});
  //   audioSource->SetVelocity(ignition::math::Vector3d::Zero);
  // }

  // caguero & nate testing: Publish a message at 1Hz toggling audio on and off.

  auto elapsed = _info.simTime - this->dataPtr->lastCommandTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed < this->dataPtr->audioPeriod)
  {
    return;
  }

  this->dataPtr->lastCommandTime = _info.simTime;

  auto *params = this->dataPtr->paramMsg.mutable_params();

  ignition::msgs::Any uriValue;
  uriValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_STRING);
  uriValue.set_string_value("drama.wav");

  // Set the uri field.
  (*params)["uri"] = uriValue;

  ignition::msgs::Any playbackValue;
  playbackValue.set_type(ignition::msgs::Any_ValueType::Any_ValueType_BOOLEAN);
  playbackValue.set_bool_value(this->dataPtr->playback);
  this->dataPtr->playback = !this->dataPtr->playback;

  // Set the playback field.
  (*params)["playback"] = playbackValue;

  this->dataPtr->audioPub.Publish(this->dataPtr->paramMsg);
}


IGNITION_ADD_PLUGIN(AudioPlayer,
                    ignition::gazebo::System,
                    AudioPlayer::ISystemConfigure,
                    AudioPlayer::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(AudioPlayer,
                          "ignition::gazebo::systems::AudioPlayer")