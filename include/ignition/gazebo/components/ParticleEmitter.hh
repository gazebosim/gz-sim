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
#ifndef IGNITION_GAZEBO_COMPONENTS_PARTICLEEMITTER_HH_
#define IGNITION_GAZEBO_COMPONENTS_PARTICLEEMITTER_HH_

#include <ignition/msgs/particle_emitter.pb.h>

#include <istream>
#include <ostream>

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace particles
{
  /// \brief Properties of a particle emitter.
  class Emitter
  {
    /// \brief Equality operator. This function checks if the given
    /// emitter has identical id than this object.
    /// \param[in] _emitter The emitter to compare against.
    /// \return True if this object matches the provided object.
    public: bool operator==(const Emitter &_emitter) const
    {
      return this->data.id() == _emitter.data.id();
    }

    /// \brief Inequality operator. This function checks if the given
    /// emitter has identical id than this object.
    /// \param[in] _emitter The emitter to compare against.
    /// \return True if this object doesn't match the provided object.
    public: bool operator!=(const Emitter &_emitter) const
    {
      return !(*this == _emitter);
    }

    /// \brief The particle emitter internal data.
    public: ignition::msgs::ParticleEmitter data;
  };
}

namespace serializers
{
  /// \brief Serializer for components::ParticleEmitter object
  class ParticleEmitterSerializer
  {
    /// \brief Serialization for particles::Emitter
    /// \param[out] _out Output stream
    /// \param[in] _emitter Object for the stream
    /// \return The stream
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const particles::Emitter &_emitter)
    {
      if (!_emitter.data.SerializeToOstream(&_out))
      {
        ignerr << "Error serializing particle emitter: " << std::endl;
        ignerr << _emitter.data.DebugString() << std::endl;
      }

      return _out;
    }

    /// \brief Deserialization for particles::Emitter
    /// \param[in] _in Input stream
    /// \param[out] _emitter The object to populate
    /// \return The stream
    public: static std::istream &Deserialize(std::istream &_in,
                                             particles::Emitter &_emitter)
    {
      if (!_emitter.data.ParseFromIstream(&_in))
      {
        ignerr << "Error deserializing particle emitter: " << std::endl;
        ignerr << _emitter.data.DebugString() << std::endl;
      }

      return _in;
    }
  };
}

// using separate namespace blocks so all components appear in Doxygen
// (appears as if Doxygen can't parse multiple components in a single
// namespace block since IGN_GAZEBO_REGISTER_COMPONENT doesn't have a
// trailing semicolon)
namespace components
{
  /// \brief A component that contains a particle emitter, which is
  /// represented by particles::Emitter
  using ParticleEmitter = Component<particles::Emitter,
        class ParticleEmitterTag,
        serializers::ParticleEmitterSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.ParticleEmitter",
      ParticleEmitter)
}
}
}
}

#endif
