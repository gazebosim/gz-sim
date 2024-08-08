/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "LogicalAudioSensorPlugin.hh"

#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>

#include <gz/sim/components/LogicalAudio.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>
#include "LogicalAudio.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LogicalAudioSensorPluginPrivate
{
  /// \brief Creates an audio source with attributes specified in an SDF file.
  /// \param[in] _elem A pointer to the source element in the SDF file.
  /// \param[in] _parent The source element's parent entity.
  /// \param[in] _ecm The simulation's EntityComponentManager.
  /// \param[in] _sdfEntityCreator An SdfEntityCreator.
  /// \param[in] _ids A list of audio source IDs that are connected to _parent.
  public: void CreateAudioSource(const sdf::ElementPtr &_elem,
              const Entity &_parent,
              EntityComponentManager &_ecm,
              SdfEntityCreator &_sdfEntityCreator,
              std::unordered_set<unsigned int> &_ids);

  /// \brief Creates a microphone with attributes specified in an SDF file.
  /// \param[in] _elem A pointer to the microphone element in the SDF file.
  /// \param[in] _parent The microphone element's parent entity.
  /// \param[in] _ecm The simulation's EntityComponentManager.
  /// \param[in] _sdfEntityCreator An SdfEntityCreator.
  /// \param[in] _ids A list of microphone IDs that are connected to _parent.
  public: void CreateMicrophone(const sdf::ElementPtr &_elem,
              const Entity &_parent,
              EntityComponentManager &_ecm,
              SdfEntityCreator &_sdfEntityCreator,
              std::unordered_set<unsigned int> &_ids);

  /// \brief Checks if a source has exceeded its play duration.
  /// \param[in] _simTimeInfo Information about the current simulation time.
  /// \param[in] _sourcePlayInfo The source's playing information.
  /// \returns true if the source's play duration has been exceeded,
  /// false otherwise
  public: bool DurationExceeded(const UpdateInfo &_simTimeInfo,
               const logical_audio::SourcePlayInfo &_sourcePlayInfo);

  /// \brief Node used to create publishers and services
  public: transport::Node node;

  /// \brief A flag used to initialize a source's playing information
  /// before starting simulation.
  public: bool firstTime{true};

  /// \brief A list of source entities for a specific parent entity
  /// (an entity can have multiple sources attached to it).
  /// The value is a pair of booleans that indicate if the source
  /// in the corresponding key should be played/stopped because of a service
  /// call. The first element in the pair indicates if the play source
  /// service was called, and the second element in the pair indicates if
  /// the stop source service was called.
  public: std::unordered_map<Entity, std::pair<bool, bool>> sourceEntities;

  /// \brief A list of microphone entities for a specific parent entity
  /// (an entity can have multiple microphones attached to it).
  /// The value is the microphone's detection publisher.
  public: std::unordered_map<Entity,
            transport::Node::Publisher> micEntities;

  /// \brief A mutex used to ensure that the play source service call does
  /// not interfere with the source's state in the PreUpdate step.
  public: std::mutex playSourceMutex;

  /// \brief A mutex used to ensure that the stop source service call does
  /// not interfere with the source's state in the PreUpdate step.
  public: std::mutex stopSourceMutex;
};

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::LogicalAudioSensorPlugin()
  : System(), dataPtr(std::make_unique<LogicalAudioSensorPluginPrivate>())
{
}

//////////////////////////////////////////////////
LogicalAudioSensorPlugin::~LogicalAudioSensorPlugin()
{
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr)
{
  const std::string kSource = "source";
  const std::string kMicrophone = "microphone";

  SdfEntityCreator sdfEntityCreator(_ecm, _eventMgr);

  const auto sdfClone = _sdf->Clone();

  if (sdfClone->HasElement(kSource))
  {
    std::unordered_set<unsigned int> allIDs;
    for (auto sourceElem = sdfClone->GetElement(kSource); sourceElem;
          sourceElem = sourceElem->GetNextElement(kSource))
    {
      this->dataPtr->CreateAudioSource(sourceElem, _entity, _ecm,
          sdfEntityCreator, allIDs);
    }
  }

  if (sdfClone->HasElement(kMicrophone))
  {
    std::unordered_set<unsigned int> allIDs;
    for (auto micElem = sdfClone->GetElement(kMicrophone); micElem;
          micElem = micElem->GetNextElement(kMicrophone))
    {
      this->dataPtr->CreateMicrophone(micElem, _entity, _ecm, sdfEntityCreator,
          allIDs);
    }
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PreUpdate(const UpdateInfo &_info,
                EntityComponentManager &_ecm)
{
  for (auto & [entity, serviceFlags] : this->dataPtr->sourceEntities)
  {
    auto& playInfo = _ecm.Component<components::LogicalAudioSourcePlayInfo>(
        entity)->Data();

    // configure the source's play information before starting the simulation
    if (this->dataPtr->firstTime)
      playInfo.startTime = _info.simTime;

    // start playing a source if the play source service was called
    std::unique_lock<std::mutex> play_lock(this->dataPtr->playSourceMutex);
    if (serviceFlags.first)
    {
      // only reset the source's play start time if it isn't playing already
      // (calling the play service on a source that's already playing does
      // nothing)
      if (!playInfo.playing)
        playInfo.startTime = _info.simTime;

      playInfo.playing = true;
      serviceFlags.first = false;
    }
    play_lock.unlock();

    // stop playing a source if the stop source service was called
    std::unique_lock<std::mutex> stop_lock(this->dataPtr->stopSourceMutex);
    if (serviceFlags.second)
    {
      playInfo.playing = false;
      serviceFlags.second = false;
    }
    stop_lock.unlock();

    // stop playing a source if the play duration has been exceeded
    if (this->dataPtr->DurationExceeded(_info, playInfo))
      playInfo.playing = false;
  }

  this->dataPtr->firstTime = false;
}

//////////////////////////////////////////////////
void LogicalAudioSensorPlugin::PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm)
{
  // get the current sim time so that it can be placed in the header
  // of microphone detection messages
  const auto simSeconds =
    std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
  const auto simNanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime);
  const auto nanosecondOffset = (simNanoseconds - simSeconds).count();

  for (auto & [micEntity, detectionPub] : this->dataPtr->micEntities)
  {
    const auto micPose = worldPose(micEntity, _ecm);
    const auto micInfo = _ecm.Component<components::LogicalMicrophone>(
        micEntity)->Data();

    _ecm.Each<components::LogicalAudioSource,
              components::LogicalAudioSourcePlayInfo>(
      [&, &publisher = detectionPub](const Entity &_entity,
          const components::LogicalAudioSource *_source,
          const components::LogicalAudioSourcePlayInfo *_playInfo)
      {
        const auto sourcePose = worldPose(_entity, _ecm);
        const auto vol = logical_audio::computeVolume(
            _playInfo->Data().playing,
            _source->Data().attFunc,
            _source->Data().attShape,
            _source->Data().emissionVolume,
            _source->Data().innerRadius,
            _source->Data().falloffDistance,
            sourcePose,
            micPose);

        if (logical_audio::detect(vol, micInfo.volumeDetectionThreshold))
        {
          // publish the source that the microphone heard, along with the
          // volume level the microphone detected. The detected source's
          // ID is embedded in the message's header
          msgs::Double msg;
          auto header = msg.mutable_header();
          auto timeStamp = header->mutable_stamp();
          timeStamp->set_sec(simSeconds.count());
          timeStamp->set_nsec(nanosecondOffset);
          auto headerData = header->add_data();
          headerData->set_key(scopedName(_entity, _ecm));
          msg.set_data(vol);

          publisher.Publish(msg);
        }

        return true;
      });
  }
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateAudioSource(
    const sdf::ElementPtr &_elem,
    const Entity &_parent,
    EntityComponentManager &_ecm,
    SdfEntityCreator &_sdfEntityCreator,
    std::unordered_set<unsigned int> &_ids)
{
  static const std::string kSourceSkipMsg =
    "Skipping the creation of this source.\n";

  if (!_elem->HasElement("id"))
  {
    gzerr << "Audio source is missing an id. " << kSourceSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  // make sure no other sources exist with the same ID in the parent
  if (_ids.find(id) != _ids.end())
  {
    gzerr << "The specified source ID of " << id << " already exists for "
      << "another source in entity " << _parent << ". " << kSourceSkipMsg;
    return;
  }
  _ids.insert(id);

  math::Pose3d pose;
  if (!_elem->HasElement("pose"))
  {
    gzwarn << "Audio source is missing a pose. "
      << "{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} will be used.\n";
    pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  else
  {
    pose = _elem->Get<math::Pose3d>("pose");
  }

  if (!_elem->HasElement("attenuation_function"))
  {
    gzerr << "Audio source has no attenuation function. " << kSourceSkipMsg;
    return;
  }
  const auto attenuationFunc = _elem->Get<std::string>("attenuation_function");

  if (!_elem->HasElement("attenuation_shape"))
  {
    gzerr << "Audio source has no attenuation shape. " << kSourceSkipMsg;
    return;
  }
  const auto attenuationShape = _elem->Get<std::string>("attenuation_shape");

  if (!_elem->HasElement("inner_radius"))
  {
    gzerr << "Audio source has no inner radius. " << kSourceSkipMsg;
    return;
  }
  const auto innerRadius = _elem->Get<double>("inner_radius");

  if (!_elem->HasElement("falloff_distance"))
  {
    gzerr << "Audio source is missing a falloff distance. " << kSourceSkipMsg;
    return;
  }
  const auto falloffDistance = _elem->Get<double>("falloff_distance");

  if (!_elem->HasElement("volume_level"))
  {
    gzerr << "Audio source is missing a volume level. " << kSourceSkipMsg;
    return;
  }
  const auto volumeLevel = _elem->Get<double>("volume_level");

  if (!_elem->HasElement("playing"))
  {
    gzerr << "Audio source is missing the playing attribute. "
           << kSourceSkipMsg;
    return;
  }
  const auto playing = _elem->Get<bool>("playing");

  if (!_elem->HasElement("play_duration"))
  {
    gzerr << "Audio source is missing the play duration. " << kSourceSkipMsg;
    return;
  }
  const auto playDuration = _elem->Get<unsigned int>("play_duration");

  // create an audio source entity
  auto entity = _ecm.CreateEntity();
  if (entity == kNullEntity)
  {
    gzerr << "Failed to create a logical audio source entity. "
      << kSourceSkipMsg;
    return;
  }
  _sdfEntityCreator.SetParent(entity, _parent);
  _ecm.CreateComponent(entity,
      components::Name("source_" + std::to_string(id)));
  _ecm.CreateComponent(entity, components::Sensor());

  // save the audio source properties as a component
  logical_audio::Source source;
  source.id = id;
  logical_audio::setAttenuationFunction(source.attFunc, attenuationFunc);
  logical_audio::setAttenuationShape(source.attShape, attenuationShape);
  source.innerRadius = innerRadius;
  source.falloffDistance = falloffDistance;
  logical_audio::validateInnerRadiusAndFalloffDistance(
      source.innerRadius,
      source.falloffDistance);
  source.emissionVolume = volumeLevel;
  logical_audio::validateVolumeLevel(source.emissionVolume);
  _ecm.CreateComponent(entity, components::LogicalAudioSource(source));

  // save the source's pose as a component
  _ecm.CreateComponent(entity,
      components::Pose(pose));

  // save the source's playing information as a component
  logical_audio::SourcePlayInfo playInfo;
  playInfo.playing = playing;
  playInfo.playDuration = std::chrono::seconds(playDuration);
  _ecm.CreateComponent(entity,
      components::LogicalAudioSourcePlayInfo(playInfo));

  // create service callbacks that allow this source to be played/stopped
  std::function<bool(msgs::Boolean &)> playSrvCb =
    [this, entity](msgs::Boolean &_resp)
    {
      std::lock_guard<std::mutex> lock(this->playSourceMutex);
      this->sourceEntities[entity].first = true;
      _resp.set_data(true);
      return true;
    };
  std::function<bool(msgs::Boolean &)> stopSrvCb =
    [this, entity](msgs::Boolean &_resp)
    {
      std::lock_guard<std::mutex> lock(this->stopSourceMutex);
      this->sourceEntities[entity].second = true;
      _resp.set_data(true);
      return true;
    };

  // create services for this source
  const auto validName = topicFromScopedName(entity, _ecm, false);
  if (validName.empty())
  {
    gzerr << "Failed to create valid topics with entity scoped name ["
           << scopedName(entity, _ecm) << "]" << std::endl;
    return;
  }
  if (!this->node.Advertise(validName + "/play", playSrvCb))
  {
    gzerr << "Error advertising the play source service for source "
      << id << " in entity " << _parent << ". " << kSourceSkipMsg;
    return;
  }
  if (!this->node.Advertise(validName + "/stop", stopSrvCb))
  {
    gzerr << "Error advertising the stop source service for source "
      << id << " in entity " << _parent << ". " << kSourceSkipMsg;
    return;
  }

  this->sourceEntities.insert({entity, {false, false}});
}

//////////////////////////////////////////////////
void LogicalAudioSensorPluginPrivate::CreateMicrophone(
    const sdf::ElementPtr &_elem,
    const Entity &_parent,
    EntityComponentManager &_ecm,
    SdfEntityCreator &_sdfEntityCreator,
    std::unordered_set<unsigned int> &_ids)
{
  static const std::string kMicSkipMsg =
    "Skipping the creation of this microphone.\n";

  if (!_elem->HasElement("id"))
  {
    gzerr << "Microphone is missing an id. " << kMicSkipMsg;
    return;
  }
  const auto id = _elem->Get<unsigned int>("id");

  // make sure no other microphones exist with the same ID in the parent
  if (_ids.find(id) != _ids.end())
  {
    gzerr << "The specified microphone ID of " << id << " already exists for "
      << "another microphone in entity " << _parent << ". " << kMicSkipMsg;
    return;
  }
  _ids.insert(id);

  math::Pose3d pose;
  if (!_elem->HasElement("pose"))
  {
    gzwarn << "Microphone is missing a pose. "
      << "{0.0, 0.0, 0.0, 0.0, 0.0, 0.0} will be used.\n";
    pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  }
  else
  {
    pose = _elem->Get<math::Pose3d>("pose");
  }

  double volumeDetectionThreshold;
  if (!_elem->HasElement("volume_threshold"))
  {
    gzwarn << "Microphone is missing a volume threshold. 0.0 will be used.\n";
    volumeDetectionThreshold = 0.0;
  }
  else
  {
    volumeDetectionThreshold = _elem->Get<double>("volume_threshold");
  }

  // create a microphone entity
  auto entity = _ecm.CreateEntity();
  if (entity == kNullEntity)
  {
    gzerr << "Failed to create a logical audio microphone entity. "
      << kMicSkipMsg;
    return;
  }
  _sdfEntityCreator.SetParent(entity, _parent);
  _ecm.CreateComponent(entity,
      components::Name("mic_" + std::to_string(id)));
  _ecm.CreateComponent(entity, components::Sensor());

  // save the microphone properties as a component
  logical_audio::Microphone microphone;
  microphone.id = id;
  microphone.volumeDetectionThreshold = volumeDetectionThreshold;
  _ecm.CreateComponent(entity, components::LogicalMicrophone(microphone));

  // save the microphone's pose as a component
  _ecm.CreateComponent(entity,
      components::Pose(pose));

  // create the detection publisher for this microphone
  auto pub = this->node.Advertise<msgs::Double>(
      topicFromScopedName(entity, _ecm, false) + "/detection");
  if (!pub)
  {
    gzerr << "Error creating a detection publisher for microphone "
      << id << " in entity " << _parent << ". " << kMicSkipMsg;
    return;
  }

  this->micEntities.insert({entity, pub});
}

//////////////////////////////////////////////////
bool LogicalAudioSensorPluginPrivate::DurationExceeded(
    const UpdateInfo &_simTimeInfo,
    const logical_audio::SourcePlayInfo &_sourcePlayInfo)
{
  auto currDuration = _simTimeInfo.simTime - _sourcePlayInfo.startTime;

  // make sure the source doesn't have an infinite play duration
  return (_sourcePlayInfo.playDuration.count() > 0 ) &&
    (currDuration > _sourcePlayInfo.playDuration);
}

GZ_ADD_PLUGIN(LogicalAudioSensorPlugin,
                    System,
                    LogicalAudioSensorPlugin::ISystemConfigure,
                    LogicalAudioSensorPlugin::ISystemPreUpdate,
                    LogicalAudioSensorPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(LogicalAudioSensorPlugin,
  "gz::sim::systems::LogicalAudioSensorPlugin")
