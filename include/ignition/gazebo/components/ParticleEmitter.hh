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
#ifndef IGNITION_GAZEBO_COMPONENTS_PARTICLEEMITTER_HH_
#define IGNITION_GAZEBO_COMPONENTS_PARTICLEEMITTER_HH_

#include <cstdint>
#include <istream>
#include <ostream>
#include <string>

#include <ignition/math/Color.hh>
#include <ignition/math/Vector3.hh>
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
  /// \brief All possible emitter type.
  enum class EmitterType {POINT, BOX, CYLINDER, ELLIPSOID};

  /// \brief Properties of a particle emitter.
  /// A particle emitter also has:
  ///    * a pose, which can be stored as a component of
  ///      an emitter entity via ignition::gazebo::components::Pose
  ///    * a material, which can be stored as a component of
  ///      an emitter entity via ignition::gazebo::components::Material
  struct Emitter
  {
    /// \brief The particle emitter's id.
    unsigned int id;

    /// \brief The emitter's name.
    std::string name;

    /// \brief The emitter's type.
    particles::EmitterType type;

    /// The size of the emitter where the particles are sampled.
    ignition::math::Vector3d size;

    /// \brief How many particles per second should be emitted.
    double rate;

    /// \brief The number of seconds the emitter is active.
    double duration;

    /// \brief Whether particle emitter is enabled or not.
    bool emitting;

    /// \brief The particle dimensions (width, height, depth).
    ignition::math::Vector3d particleSize;

    /// \brief The number of seconds each particle will ’live’ for before
    /// being destroyed.
    double lifetime;

    /// \brief The minimum velocity each particle is emitted (m/s).
    double minVelocity;

    /// \brief The maximum velocity each particle is emitted (m/s).
    double maxVelocity;

    /// \brief The starting color of the particles.
    ignition::math::Color colorStart;

    /// \brief The end color of the particles.
    ignition::math::Color colorEnd;

    /// \brief The amount by which to scale the particles in both x and y
    /// direction per second.
    double scaleRate;

    /// \brief The path to the color image used as an affector.
    std::string colorRangeImage;

    public: bool operator==(const Emitter &_emitter) const
    {
      return this->id == _emitter.id;
    }

    public: bool operator!=(const Emitter &_emitter) const
    {
      return !(*this == _emitter);
    }
  };
}

namespace serializers
{
  /// \brief Output stream overload for particles::EmitterType
  inline std::ostream &operator<<(std::ostream &_out,
                                  const particles::EmitterType &_type)
  {
    auto temp = static_cast<unsigned int>(_type);
    _out << temp;
    return _out;
  }

  /// \brief Input stream overload for particles::EmitterType
  inline std::istream &operator>>(std::istream &_in,
                                  particles::EmitterType &_type)
  {
    unsigned int temp = 0u;
    if (_in >> temp)
      _type = static_cast<particles::EmitterType>(temp);
    return _in;
  }

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
      _out << _emitter.id << " " << _emitter.name << " " << _emitter.type
           << " " << _emitter.size << " " << _emitter.rate
           << " " << _emitter.duration << " " << _emitter.emitting
           << " " << _emitter.particleSize << " " << _emitter.lifetime
           << " " << _emitter.minVelocity << " " << _emitter.maxVelocity
           << " " << _emitter.colorStart << " " << _emitter.colorEnd
           << " " << _emitter.scaleRate << " " << _emitter.colorRangeImage;
      return _out;
    }

    /// \brief Deserialization for particles::Emitter
    /// \param[in] _in Input stream
    /// \param[out] _emitter The object to populate
    /// \return The stream
    public: static std::istream &Deserialize(std::istream &_in,
                                             particles::Emitter &_emitter)
    {
      _in >> _emitter.id >> _emitter.name >> _emitter.type
          >> _emitter.size >> _emitter.rate >> _emitter.duration
          >> _emitter.emitting >> _emitter.particleSize >> _emitter.lifetime
          >> _emitter.minVelocity >> _emitter.maxVelocity >> _emitter.colorStart
          >> _emitter.colorEnd >> _emitter.scaleRate
          >> _emitter.colorRangeImage;
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
