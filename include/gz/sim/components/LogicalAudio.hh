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
#ifndef GZ_SIM_COMPONENTS_LOGICALAUDIO_HH_
#define GZ_SIM_COMPONENTS_LOGICALAUDIO_HH_

#include <chrono>
#include <cstdint>
#include <istream>
#include <ostream>
#include <ratio>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace logical_audio
{
  /// \brief Audio source attenuation functions.
  /// AttenuationFunction::Undefined is used to indicate that an
  /// attenuation function has not been defined yet.
  enum class AttenuationFunction { LINEAR, UNDEFINED };

  /// \brief Audio source attenuation shapes.
  /// AttenuationShape::Undefined is used to indicate that an
  /// attenuation shape has not been defined yet.
  enum class AttenuationShape { SPHERE, UNDEFINED };

  /// \brief Properties of a logical audio source.
  /// A source also has a pose, which can be stored as a component of a
  /// source entity via gz::sim::components::Pose
  struct Source
  {
    /// \brief The source's id
    unsigned int id;

    /// \brief The source's attenuation function
    logical_audio::AttenuationFunction attFunc;

    /// \brief The source's attenuation shape
    logical_audio::AttenuationShape attShape;

    /// \brief The source's inner radius, which should be >= 0
    double innerRadius;

    /// \brief The source's falloff distance, which should be
    /// greater than Source::innerRadius
    double falloffDistance;

    /// \brief The source's emission volume, which should be
    /// between 0.0 (0% volume) and 1.0 (100% volume)
    double emissionVolume;

    public: bool operator==(const Source &_source) const
    {
      return this->id == _source.id;
    }

    public: bool operator!=(const Source &_source) const
    {
      return !(*this == _source);
    }
  };

  /// \brief A source's playing information.
  /// Useful for keeping track of when to start/stop playing a source.
  struct SourcePlayInfo
  {
    /// \brief Constructor
    SourcePlayInfo() : startTime()
    {
    }

    /// \brief Whether the source is currently playing or not
    bool playing{false};

    /// \brief How long the source should play for, in seconds.
    /// Setting this to 0 means the source has a play duration of infinity
    std::chrono::seconds playDuration;

    /// \brief The time at which the source most recently started playing
    std::chrono::steady_clock::duration startTime;

    public: bool operator==(const SourcePlayInfo &_sourcePlayInfo) const
    {
      return (this->playing == _sourcePlayInfo.playing) &&
             (this->playDuration == _sourcePlayInfo.playDuration) &&
             (this->startTime == _sourcePlayInfo.startTime);
    }

    public: bool operator!=(const SourcePlayInfo &_sourcePlayInfo) const
    {
      return !(*this == _sourcePlayInfo);
    }
  };

  /// \brief Properties of a logical audio microphone.
  /// A microphone also has a pose, which can be stored as a component of a
  /// microphone entity via gz::sim::components::Pose
  struct Microphone
  {
    /// \brief The microphone's id
    unsigned int id;

    /// \brief The minimum volume this microphone can detect.
    /// This should be a value between 0.0 (0% volume) and 1.0 (100% volume)
    double volumeDetectionThreshold;

    public: bool operator==(const Microphone &_microphone) const
    {
      return this->id == _microphone.id;
    }

    public: bool operator!=(const Microphone &_microphone) const
    {
      return !(*this == _microphone);
    }
  };
}

namespace serializers
{
  /// \brief Output stream overload for logical_audio::AttenuationFunction
  inline std::ostream &operator<<(std::ostream &_out,
              const logical_audio::AttenuationFunction &_func)
  {
    auto temp = static_cast<unsigned int>(_func);
    _out << temp;
    return _out;
  }

  /// \brief Input stream overload for logical_audio::AttenuationFunction
  inline std::istream &operator>>(std::istream &_in,
              logical_audio::AttenuationFunction &_func)
  {
    unsigned int temp = 0u;
    if (_in >> temp)
      _func = static_cast<logical_audio::AttenuationFunction>(temp);
    return _in;
  }

  /// \brief Output stream overload for logical_audio::AttenuationShape
  inline std::ostream &operator<<(std::ostream &_out,
              const logical_audio::AttenuationShape &_shape)
  {
    auto temp = static_cast<unsigned int>(_shape);
    _out << temp;
    return _out;
  }

  /// \brief Input stream overload for logical_audio::AttenuationShape
  inline std::istream &operator>>(std::istream &_in,
              logical_audio::AttenuationShape &_shape)
  {
    unsigned int temp = 0u;
    if (_in >> temp)
       _shape = static_cast<logical_audio::AttenuationShape>(temp);
    return _in;
  }

  /// \brief Output stream overload for std::chrono::steady_clock::duration
  inline std::ostream &operator<<(std::ostream &_out,
      const std::chrono::steady_clock::duration &_dur)
  {
    _out << std::chrono::duration_cast<std::chrono::nanoseconds>(
        _dur).count();
    return _out;
  }

  /// \brief Input stream overload for std::chrono::steady_clock::duration
  inline std::istream &operator>>(std::istream &_in,
      std::chrono::steady_clock::duration &_dur)
  {
    int64_t time;
    _in >> time;
    _dur = std::chrono::duration<int64_t, std::nano>(time);
    return _in;
  }

  /// \brief Serializer for components::LogicalAudioSource object
  class LogicalAudioSourceSerializer
  {
    /// \brief Serialization for logical_audio::Source
    /// \param[out] _out Output stream
    /// \param[in] _source Object for the stream
    /// \return The stream
    public: static std::ostream &Serialize(std::ostream &_out,
                const logical_audio::Source &_source)
    {
      _out << _source.id << " " << _source.attFunc << " " << _source.attShape
        << " " << _source.innerRadius << " " << _source.falloffDistance
        << " " << _source.emissionVolume;
      return _out;
    }

    /// \brief Deserialization for logical_audio::Source
    /// \param[in] _in Input stream
    /// \param[out] _source The object to populate
    /// \return The stream
    public: static std::istream &Deserialize(std::istream &_in,
                logical_audio::Source &_source)
    {
      _in >> _source.id >> _source.attFunc >> _source.attShape
        >> _source.innerRadius >> _source.falloffDistance
        >> _source.emissionVolume;
      return _in;
    }
  };

  /// \brief Serializer for components::LogicalAudioSourcePlayInfo object
  class LogicalAudioSourcePlayInfoSerializer
  {
    /// \brief Serialization for logical_audio::SourcePlayInfo
    /// \param[out] _out Output stream
    /// \param[in] _playInfo Object for the stream
    /// \return The stream
    public: static std::ostream &Serialize(std::ostream &_out,
                const logical_audio::SourcePlayInfo &_playInfo)
    {
      _out << _playInfo.playing << " " << _playInfo.playDuration.count() << " "
        << _playInfo.startTime;
      return _out;
    }

    /// \brief Deserialization for logical_audio::SourcePlayInfo
    /// \param[in] _in Input stream
    /// \param[out] _playInfo The object to populate
    /// \return The stream
    public: static std::istream &Deserialize(std::istream &_in,
                logical_audio::SourcePlayInfo &_playInfo)
    {
      uint64_t count;
      _in >> _playInfo.playing >> count >> _playInfo.startTime;
      _playInfo.playDuration = std::chrono::seconds(count);
      return _in;
    }
  };

  /// \brief Serializer for components::LogicalMicrophone object
  class LogicalMicrophoneSerializer
  {
    /// \brief Serialization for logical_audio::Microphone
    /// \param[out] _out Output stream
    /// \param[in] _mic Object for the stream
    /// \return The stream
    public: static std::ostream &Serialize(std::ostream &_out,
                const logical_audio::Microphone &_mic)
    {
      _out << _mic.id << " " << _mic.volumeDetectionThreshold;
      return _out;
    }

    /// \brief Deserialization for logical_audio::Microphone
    /// \param[in] _in Inout stream
    /// \param[out] _mic The object to populate
    public: static std::istream &Deserialize(std::istream &_in,
                logical_audio::Microphone &_mic)
    {
      _in >> _mic.id >> _mic.volumeDetectionThreshold;
      return _in;
    }
  };
}

// using separate namespace blocks so all components appear in Doxygen
// (appears as if Doxygen can't parse multiple components in a single
// namespace block since GZ_SIM_REGISTER_COMPONENT doesn't have a
// trailing semicolon)
namespace components
{
  /// \brief A component that contains a logical audio source, which is
  /// represented by logical_audio::Source
  using LogicalAudioSource = Component<logical_audio::Source,
        class LogicalAudioSourceTag,
        serializers::LogicalAudioSourceSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.LogicalAudioSource",
      LogicalAudioSource)
}

namespace components
{
  /// \brief A component that contains a logical audio source's playing
  /// information, which is represented by logical_audio::SourcePlayInfo
  using LogicalAudioSourcePlayInfo = Component<logical_audio::SourcePlayInfo,
        class LogicalAudioSourcePlayInfoTag,
        serializers::LogicalAudioSourcePlayInfoSerializer>;
  GZ_SIM_REGISTER_COMPONENT(
      "gz_sim_components.LogicalAudioSourcePlayInfo",
      LogicalAudioSourcePlayInfo)
}

namespace components
{
  /// \brief A component that contains a logical microphone, which is
  /// represented by logical_audio::Microphone
  using LogicalMicrophone = Component<logical_audio::Microphone,
        class LogicalMicrophoneTag,
        serializers::LogicalMicrophoneSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.LogicalMicrophone",
      LogicalMicrophone)
}
}
}
}

#endif
