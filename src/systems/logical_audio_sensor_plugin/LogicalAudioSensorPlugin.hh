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
#ifndef GZ_SIM_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_HH_
#define GZ_SIM_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward Declaration
  class LogicalAudioSensorPluginPrivate;

  /// \brief A plugin for logical audio detection.
  ///
  /// Each `<plugin>` tag can accept multiple sensors (sound sources
  /// and/or microphones).
  /// After each simulation step, microphones check if audio
  /// was detected by any sources in the world.
  /// No audio is actually played to an audio device
  /// such as speakers. This plugin is meant to check if audio
  /// could theoretically be heard at a certain location or not.
  ///
  /// ## System Parameters
  ///
  /// Secifying an audio source via SDF is done as follows:
  ///
  /// - `<source>` A new audio source in the environment, which has the
  ///   following child elements:
  ///     + `<id>` The source ID, which must be unique and an integer >= 0.
  ///       An ID < 0 results in undefined behavior.
  ///       The ID must be unique within the scope of all other source IDs
  ///       in the plugin's parent element - so, if a source was created in a
  ///       `<model>` with an ID of 1, no other sources in the same model can
  ///        have an ID of 1 (however, sources that belong to other models can
  ///        have an ID of 1).
  ///     + `<pose>` The pose, expressed as "x y z roll pitch yaw".
  ///       Each pose coordinate must be separated by whitespace.
  ///       The pose is defined relative to the plugin's parent element.
  ///       So, if the plugin is used inside of a `<model>` tag, then the
  ///       source's `<pose>` is relative to the model's pose.
  ///       If the plugin is used inside of a `<world>` tag, then the source's
  ///       `<pose>` is relative to the world (i.e., `<pose>` specifies an
  ///       absolute pose).
  ///       If no pose is specified, {0, 0, 0, 0, 0, 0} will be used.
  ///     + `<attenuation_function>` The attenuation function.
  ///       See logical_audio::AttenuationFunction for a list of valid
  ///       attenuation functions, and logical_audio::SetAttenuationFunction
  ///       for how to specify an attenuation function in SDF.
  ///     + `<attenuation_shape>` The attenuation shape.
  ///       See logical_audio::AttenuationShape for a list of valid
  ///       attenuation shapes, and logical_audio::SetAttenuationShape for how
  ///       to specify an attenuation shape in SDF.
  ///     + `<inner_radius>` The inner radius of the attenuation shape.
  ///       This value must be >= 0.0. The volume of the source will be
  ///       `<source><volume>` at locations that have a distance <= inner
  ///       radius from the source.
  ///     + `<falloff_distance>` The falloff distance. This value must be
  ///        greater than the value of the source's `<inner_radius>`. This
  ///        defines the distance from the audio source where the volume
  ///        becomes 0.
  ///     + `<volume_level>` The volume level emitted from the source. This must
  ///       be a value between 0.0 and 1.0 (representing 0% to 100%).
  ///     + `<playing>` Whether the source should play immediately or not.
  ///       Use true to initiate audio immediately, and false otherwise.
  ///     + `<play_duration>` The duration (in seconds) audio is played from the
  ///       source. This value must be an integer >= 0.
  ///       A value of 0 means that the source will play for an infinite amount
  ///       of time.
  ///
  /// Specifying a microphone via SDF is done as follows:
  ///
  /// - `<microphone>` A new microphone in the environment,
  ///   which has the following child elements:
  ///     + `<id>` The microphone ID, which must be unique and an integer >= 0.
  ///       An ID < 0 results in undefined behavior.
  ///       The ID must be unique within the scope of all other microphone IDs
  ///       in the plugin's parent element - so, if a microphone was created in
  ///       a `<model>` with an ID of 1, no other microphones in the same model
  ///       can have an ID of 1 (however, microphones that belong to other
  ///       models can have an ID of 1).
  ///     + `<pose>` The pose, expressed as "x y z roll pitch yaw".
  ///       Each pose coordinate must be separated by whitespace.
  ///       The pose is defined relative to the plugin's parent element.
  ///       So, if the plugin is used inside of a `<model>` tag, then the
  ///       source's `<pose>`is relative to the model's pose.
  ///       If the plugin is used inside of a `<world>` tag, then the source's
  ///       `<pose>` is relative to the world (i.e., `<pose>` specifies an
  ///       absolute pose).
  ///       If no pose is specified, {0, 0, 0, 0, 0, 0} will be used.
  ///     + `<volume_threshold>` The minimum volume level the microphone
  ///       can detect. This must be a value between 0.0 and 1.0
  ///       (representing 0% to 100%).
  ///       If no volume threshold is specified, 0.0 will be used.
  ///
  /// Sources can be started and stopped via Gazebo service calls.
  /// If a source is successfully created, the following services will be
  /// created for the source (`<PREFIX>` is the scoped name for the source - see
  /// gz::sim::scopedName for more details - and `<id>` is the value
  /// specified in the source's `<id>` tag from the SDF):
  ///     * `<PREFIX>/source_<id>/play`
  ///         * Starts playing the source with the specified ID.
  ///           If the source is already playing, nothing happens.
  ///     * `<PREFIX>/source_<id>/stop`
  ///         * Stops playing the source with the specified ID.
  ///           If the source is already stopped, nothing happens.
  ///
  /// Microphone detection information can be retrieved via Gazebo topics.
  /// Whenever a microphone detects a source, it publishes a message to the
  /// `<PREFIX>/mic_<id>/detection` topic, where `<PREFIX>` is the scoped name
  /// for the microphone - see gz::sim::scopedName for more details -
  /// and `<id>` is the value specified in the microphone's `<id>` tag from the
  /// SDF.
  class LogicalAudioSensorPlugin :
    public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
  {
    /// \brief Constructor
    public: LogicalAudioSensorPlugin();

    /// \brief Destructor
    public: ~LogicalAudioSensorPlugin() override;

    /// Documentation inherited
    public: void Configure(const Entity &_entity,
                const std::shared_ptr<const sdf::Element> &_sdf,
                EntityComponentManager &_ecm,
                EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// Documentation inherited
    public: void PostUpdate(const UpdateInfo &_info,
                const EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<LogicalAudioSensorPluginPrivate> dataPtr;
  };
}
}
}
}

#endif
