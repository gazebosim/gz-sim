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
#ifndef IGNITION_GAZEBO_SYSTEMS_PARTICLE_EMITTER_HH_
#define IGNITION_GAZEBO_SYSTEMS_PARTICLE_EMITTER_HH_

#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  class ParticleEmitterPrivate;

  /// \brief A system for creating a particle emitter.
  ///
  /// System parameters
  ///
  /// `<emitter_name>`: Unique name for the particle emitter.
  /// `<type>`: The emitter type (point, box, cylinder, ellipsoid).
  /// `<pose>`: The pose of the emitter.
  /// `<size>`: The size of the emitter where the particles are sampled.
  ///           Default value is (1, 1, 1).
  ///           Note that the interpretation of the emitter area varies
  ///           depending on the emmiter type:
  ///             - point: The area is ignored.
  ///             - box: The area is interpreted as width X height X depth.
  ///             - cylinder: The area is interpreted as the bounding box of the
  ///                         cilinder. The cylinder is oriented along the
  ///                         Z-axis.
  ///             - ellipsoid: The area is interpreted as the bounding box of an
  ///                          ellipsoid shaped area, i.e. a sphere or
  ///                          squashed-sphere area. The parameters are again
  ///                          identical to EM_BOX, except that the dimensions
  ///                          describe the widest points along each of the
  ///                          axes.
  /// `<rate>`: How many particles per second should be emitted.
  /// `<duration`>: The number of seconds the emitter is active. A value of 0
  ///               means infinite duration.
  /// `<emitting>`: This is used to turn on or off particle emission.
  /// `<particle_size>`: Set the particle dimensions (width, height, depth).
  /// `<lifetime>`: Set the number of seconds each particle will ’live’ for
  ///               before being destroyed.
  /// `<material>`: Sets the material which all particles in the emitter will
  ///               use.
  /// `<min_velocity>`: Sets a minimum velocity for each particle (m/s).
  /// `<max_velocity>`: Sets a maximum velocity for each particle (m/s).
  /// `<color_start>`: Sets the starting color for all particle emitted.
  ///                  The actual color will be interpolated between this color
  ///                  and the one set under <color_end>.
  ///                  Color::White is the default color for the particles
  ///                  unless a specific function is used.
  ///                  Note that this function overrides the particle colors set
  ///                  with <ColorRangeImage>.
  /// `<color_end>`: Sets the end color for all particle emitted.
  ///                The actual color will be interpolated between this color
  ///                and the one set under <color_start>.
  ///                Color::White is the default color for the particles
  ///                unless a specific function is used.
  ///                Note that this function overrides the particle colors set
  ///                with <ColorRangeImage>.
  /// `<scale_rate>`: Sets the amount by which to scale the particles in both x
  ///                 and y direction per second.
  /// `<color_range_image>`: Sets the path to the color image used as an
  ///                        affector. This affector modifies the color of
  ///                        particles in flight. The colors are taken from a
  ///                        specified image file. The range of color values
  ///                        begins from the left side of the image and move to
  ///                        the right over the lifetime of the particle,
  ///                        therefore only the horizontal dimension of the
  ///                        image is used.
  ///                        Note that this function overrides the particle
  /// colors set with <color_start> and <color_end>.
  class IGNITION_GAZEBO_VISIBLE ParticleEmitter
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ParticleEmitter();

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ParticleEmitterPrivate> dataPtr;
  };
  }
}
}
}

#endif

