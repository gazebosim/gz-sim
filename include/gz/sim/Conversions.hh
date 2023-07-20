/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_SIM_CONVERSIONS_HH_
#define GZ_SIM_CONVERSIONS_HH_

#include <gz/msgs/actor.pb.h>
#include <gz/msgs/atmosphere.pb.h>
#include <gz/msgs/axis.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/geometry.pb.h>
#include <gz/msgs/gui.pb.h>
#include <gz/msgs/inertial.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/material.pb.h>
#include <gz/msgs/particle_emitter.pb.h>
#include <gz/msgs/plugin.pb.h>
#include <gz/msgs/plugin_v.pb.h>
#include <gz/msgs/physics.pb.h>
#include <gz/msgs/scene.pb.h>
#include <gz/msgs/sensor.pb.h>
#include <gz/msgs/sensor_noise.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <chrono>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/AxisAlignedBox.hh>
#include <gz/math/Inertial.hh>
#include <sdf/Actor.hh>
#include <sdf/Atmosphere.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Gui.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Light.hh>
#include <sdf/Material.hh>
#include <sdf/Noise.hh>
#include <sdf/ParticleEmitter.hh>
#include <sdf/Plugin.hh>
#include <sdf/Physics.hh>
#include <sdf/Projector.hh>
#include <sdf/Scene.hh>
#include <sdf/Sensor.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Types.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    /// \brief Helper function that sets a mutable msgs::SensorNoise object
    /// to the values contained in a sdf::Noise object.
    /// \param[out] _msg SensorNoise message to set.
    /// \param[in] _sdf SDF Noise object.
    void GZ_SIM_VISIBLE
    set(msgs::SensorNoise *_msg, const sdf::Noise &_sdf);

    /// \brief Helper function that sets a mutable msgs::WorldStatistics object
    /// to the values contained in a sim::UpdateInfo  object.
    /// \param[out] _msg WorldStatistics message to set.
    /// \param[in] _in UpdateInfo object.
    void GZ_SIM_VISIBLE
    set(msgs::WorldStatistics *_msg, const UpdateInfo &_in);

    /// \brief Helper function that sets a mutable msgs::Time object
    /// to the values contained in a std::chrono::steady_clock::duration
    /// object.
    /// \param[out] _msg Time message to set.
    /// \param[in] _in Chrono duration object.
    void GZ_SIM_VISIBLE
    set(msgs::Time *_msg, const std::chrono::steady_clock::duration &_in);

    /// \brief Generic conversion from an SDF geometry to another type.
    /// \param[in] _in SDF geometry.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Geometry &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF geometry to a geometry
    /// message.
    /// \param[in] _in SDF geometry.
    /// \return Geometry message.
    template<>
    msgs::Geometry convert(const sdf::Geometry &_in);

    /// \brief Generic conversion from a msgs Pose to another type.
    /// \param[in] _in msgs Pose
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Pose &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion for msgs Pose to math Pose
    /// \param[in] _in msgs Pose
    /// \return math Pose.
    template<>
    math::Pose3d convert(const msgs::Pose &_in);

    /// \brief Generic conversion from a geometry message to another type.
    /// \param[in] _in Geometry message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Geometry &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a geometry message to a geometry
    /// SDF object.
    /// \param[in] _in Geometry message.
    /// \return SDF geometry.
    template<>
    sdf::Geometry convert(const msgs::Geometry &_in);

    /// \brief Generic conversion from an SDF material to another type.
    /// \param[in] _in SDF material.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Material &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF material to a material
    /// message.
    /// \param[in] _in SDF material.
    /// \return Material message.
    template<>
    msgs::Material convert(const sdf::Material &_in);

    /// \brief Generic conversion from a material message to another type.
    /// \param[in] _in Material message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Material &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a material message to a material
    /// SDF object.
    /// \param[in] _in Material message.
    /// \return SDF material.
    template<>
    sdf::Material convert(const msgs::Material &_in);

    /// \brief Generic conversion from an SDF actor to another type.
    /// \param[in] _in SDF actor.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Actor &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF actor to an actor
    /// message.
    /// \param[in] _in SDF actor.
    /// \return Actor message.
    template<>
    msgs::Actor convert(const sdf::Actor &_in);

    /// \brief Generic conversion from an actor message to another type.
    /// \param[in] _in Actor message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Actor& _in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an actor message to an actor
    /// SDF object.
    /// \param[in] _in Actor message.
    /// \return Actor SDF object.
    template<>
    sdf::Actor convert(const msgs::Actor &_in);

    /// \brief Generic conversion from an SDF light to another type.
    /// \param[in] _in SDF light.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Light &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF light to a light
    /// message.
    /// \param[in] _in SDF light.
    /// \return Light message.
    template<>
    msgs::Light convert(const sdf::Light &_in);

    /// \brief Generic conversion from a SDF light type to string.
    /// \param[in] _in SDF light type.
    /// \return Conversion result.
    /// \tparam Out Output type.
    std::string GZ_SIM_VISIBLE
    convert(const sdf::LightType &_in);

    /// \brief Generic conversion from a light message to another type.
    /// \param[in] _in Light message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Light& _in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a light message to a light
    /// SDF object.
    /// \param[in] _in Light message.
    /// \return Light SDF object.
    template<>
    sdf::Light convert(const msgs::Light &_in);

    /// \brief Specialized conversion from a string to a sdf light type
    /// \param[in] _in String with the light type.
    /// \return Light type emun SDF object.
    sdf::LightType GZ_SIM_VISIBLE
    convert(const std::string &_in);

    /// \brief Generic conversion from an SDF gui to another type.
    /// \param[in] _in SDF gui.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Gui &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF gui to a gui message.
    /// \param[in] _in SDF gui.
    /// \return Gui message.
    template<>
    msgs::GUI convert(const sdf::Gui &_in);

    /// \brief Generic conversion from a steady clock duration to another type.
    /// \param[in] _in Steady clock duration.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const std::chrono::steady_clock::duration &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a steady clock duration to a time
    /// message.
    /// \param[in] _in Steady clock duration.
    /// \return Gazebo time message.
    template<>
    msgs::Time convert(const std::chrono::steady_clock::duration &_in);

    /// \brief Generic conversion from a time message to another type.
    /// \param[in] _in Time message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Time &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a time message to a steady clock
    /// duration.
    /// \param[in] _in Time message.
    /// \return Steady clock duration.
    template<>
    std::chrono::steady_clock::duration convert(const msgs::Time &_in);

    /// \brief Generic conversion from a math inertial to another type.
    /// \param[in] _in Math inertial.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const math::Inertiald &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a math inertial to an inertial
    /// message.
    /// \param[in] _in Math inertial.
    /// \return Inertial message.
    template<>
    msgs::Inertial convert(const math::Inertiald &_in);

    /// \brief Generic conversion from an inertial message to another type.
    /// \param[in] _in Inertial message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Inertial &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an inertial message to an inertial
    /// math object.
    /// \param[in] _in Inertial message.
    /// \return math inertial.
    template<>
    math::Inertiald convert(const msgs::Inertial &_in);

    /// \brief Generic conversion from an SDF joint axis to another type.
    /// \param[in] _in SDF joint axis.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::JointAxis &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF joint axis to an axis
    /// message.
    /// \param[in] _in SDF joint axis.
    /// \return Axis message.
    template<>
    msgs::Axis convert(const sdf::JointAxis &_in);

    /// \brief Generic conversion from an axis message to another type.
    /// \param[in] _in Axis message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Axis &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an axis message to a joint axis
    /// SDF object.
    /// \param[in] _in Axis message.
    /// \return SDF joint axis.
    template<>
    sdf::JointAxis convert(const msgs::Axis &_in);

    /// \brief Generic conversion from an SDF scene to another type.
    /// \param[in] _in SDF scene.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Scene &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF scene to a scene message
    /// \param[in] _in SDF scene.
    /// \return Scene message.
    template<>
    msgs::Scene convert(const sdf::Scene &_in);

    /// \brief Generic conversion from a scene message to another type.
    /// \param[in] _in Scene  message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Scene &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a scene message to a scene
    /// SDF object.
    /// \param[in] _in Scene message.
    /// \return SDF scene.
    template<>
    sdf::Scene convert(const msgs::Scene &_in);

    /// \brief Generic conversion from an SDF atmosphere to another type.
    /// \param[in] _in SDF atmosphere.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Atmosphere &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF atmosphere to an atmosphere
    /// message
    /// \param[in] _in SDF atmosphere.
    /// \return Atmosphere message.
    template<>
    msgs::Atmosphere convert(const sdf::Atmosphere &_in);

    /// \brief Generic conversion from an atmosphere message to another type.
    /// \param[in] _in Atmosphere message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Atmosphere &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an atmosphere message to an
    /// atmosphere SDF object.
    /// \param[in] _in Atmosphere message.
    /// \return SDF scene.
    template<>
    sdf::Atmosphere convert(const msgs::Atmosphere &_in);


    /// \brief Generic conversion from an SDF Physics to another type.
    /// \param[in] _in SDF Physics.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Physics &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF physics to a physics
    /// message.
    /// \param[in] _in SDF physics.
    /// \return Physics message.
    template<>
    msgs::Physics convert(const sdf::Physics &_in);

    /// \brief Generic conversion from a physics message to another type.
    /// \param[in] _in Physics message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Physics &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a physics message to a physics
    /// SDF object.
    /// \param[in] _in Physics message.
    /// \return SDF physics.
    template<>
    sdf::Physics convert(const msgs::Physics &_in);


    /// \brief Generic conversion from an SDF Sensor to another type.
    /// \param[in] _in SDF Sensor.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Sensor &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF sensor to a sensor
    /// message.
    /// \param[in] _in SDF sensor.
    /// \return Sensor message.
    template<>
    msgs::Sensor convert(const sdf::Sensor &_in);

    /// \brief Generic conversion from a sensor message to another type.
    /// \param[in] _in Sensor message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Sensor &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a sensor message to a sensor
    /// SDF object.
    /// \param[in] _in Sensor message.
    /// \return SDF sensor.
    template<>
    sdf::Sensor convert(const msgs::Sensor &_in);

    /// \brief Generic conversion from a sensor noise message to another type.
    /// \param[in] _in SensorNoise message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::SensorNoise &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a sensor noise message to a noise
    /// SDF object.
    /// \param[in] _in Sensor noise message.
    /// \return SDF noise.
    template<>
    sdf::Noise convert(const msgs::SensorNoise &_in);

    /// \brief Generic conversion from a world statistics message to another
    /// type.
    /// \param[in] _in WorldStatistics message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::WorldStatistics &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a world statistics message to an
    /// `gz::sim::UpdateInfo` object.
    /// \param[in] _in WorldStatistics message.
    /// \return Update info.
    template<>
    UpdateInfo convert(const msgs::WorldStatistics &_in);

    /// \brief Generic conversion from update info to another type.
    /// \param[in] _in Update info.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const UpdateInfo &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from update info to a world statistics
    /// message.
    /// \param[in] _in Update info.
    /// \return World statistics message.
    template<>
    msgs::WorldStatistics convert(const UpdateInfo &_in);

    /// \brief Generic conversion from an SDF collision to another type.
    /// \param[in] _in SDF collision.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Collision &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF collision to a collision
    /// message.
    /// \param[in] _in SDF collision.
    /// \return Collision message.
    template<>
    msgs::Collision convert(const sdf::Collision &_in);

    /// \brief Generic conversion from a collision message to another type.
    /// \param[in] _in Collision message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Collision &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a collision message to a collision
    /// SDF object.
    /// \param[in] _in Collision message.
    /// \return SDF collision.
    template<>
    sdf::Collision convert(const msgs::Collision &_in);

    /// \brief Generic conversion from a string to another type.
    /// \param[in] _in string.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const std::string &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a string to an Entity_Type msg.
    /// \param[in] _in string message.
    /// \return Entity_Type.
    template<>
    msgs::Entity_Type convert(const std::string &_in);

    /// \brief Generic conversion from axis aligned box object to another type.
    /// \param[in] _in Axis aligned box object.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const math::AxisAlignedBox &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a math axis aligned box object to an
    /// axis aligned box message
    /// \param[in] _in Axis aligned box message
    /// \return Axis aligned box message.
    template<>
    msgs::AxisAlignedBox convert(const math::AxisAlignedBox &_in);

    /// \brief Generic conversion from an axis aligned box message to another
    /// type.
    /// \param[in] _in Axis aligned box message
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::AxisAlignedBox &_in)
    {
      (void)_in;
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an math axis aligned box message to
    /// an axis aligned box object.
    /// \param[in] _in Axis aligned box object
    /// \return Axis aligned box object.
    template<>
    math::AxisAlignedBox convert(const msgs::AxisAlignedBox &_in);

    /// \brief Generic conversion from a particle emitter SDF object to another
    /// type.
    /// \param[in] _in Particle emitter SDF object.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::ParticleEmitter &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a particle emitter SDF object to
    /// a particle emitter message object.
    /// \param[in] _in Particle emitter SDF object.
    /// \return Particle emitter message.
    template<>
    msgs::ParticleEmitter convert(const sdf::ParticleEmitter &_in);

    /// \brief Generic conversion from a particle emitter SDF object to another
    /// type.
    /// \param[in] _in Particle emitter SDF object.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::ParticleEmitter &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a particle emitter SDF object to
    /// a particle emitter message object.
    /// \param[in] _in Particle emitter SDF object.
    /// \return Particle emitter message.
    template<>
    sdf::ParticleEmitter convert(const msgs::ParticleEmitter &_in);

    /// \brief Generic conversion from a projector SDF object to another
    /// type.
    /// \param[in] _in Projector SDF object.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Projector &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a projector SDF object to
    /// a projector message object.
    /// \param[in] _in Projector SDF object.
    /// \return Projector message.
    template<>
    msgs::Projector convert(const sdf::Projector &_in);

    /// \brief Generic conversion from a projector SDF object to another
    /// type.
    /// \param[in] _in Projector SDF object.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Projector &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a projector SDF object to
    /// a projector message object.
    /// \param[in] _in Projecotr SDF object.
    /// \return Projector message.
    template<>
    sdf::Projector convert(const msgs::Projector &_in);


    /// \brief Generic conversion from an SDF element to another type.
    /// \param[in] _in SDF element.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Element &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF element to a plugin message.
    /// \param[in] _in SDF element.
    /// \return Plugin message.
    template<>
    msgs::Plugin convert(const sdf::Element &_in);

    /// \brief Generic conversion from an SDF plugin to another type.
    /// \param[in] _in SDF plugin.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Plugin &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF plugin to a plugin message.
    /// \param[in] _in SDF plugin.
    /// \return Plugin message.
    template<>
    msgs::Plugin convert(const sdf::Plugin &_in);

    /// \brief Generic conversion from an SDF plugins to another type.
    /// \param[in] _in SDF plugins.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Plugins &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF plugins to a plugin_v message.
    /// \param[in] _in SDF plugins.
    /// \return Plugin_V message.
    template<>
    msgs::Plugin_V convert(const sdf::Plugins &_in);

    /// \brief Generic conversion from a msgs::Plugin to another type.
    /// \param[in] _in msgs::Plugin.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Plugin &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a msgs::Plugin to an sdf::Plugin.
    /// \param[in] _in msgs::Plugin.
    /// \return sdf::Plugin.
    template<>
    sdf::Plugin convert(const msgs::Plugin &_in);

    /// \brief Generic conversion from a msgs::Plugin_V to another type.
    /// \param[in] _in msgs::Plugin_V.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Plugin_V &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a msgs::Plugin_V to an sdf::Plugins.
    /// \param[in] _in msgs::Plugin_V.
    /// \return sdf::Plugins.
    template<>
    sdf::Plugins convert(const msgs::Plugin_V &_in);
    }
  }
}
#endif
