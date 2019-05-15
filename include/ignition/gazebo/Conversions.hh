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
#ifndef IGNITION_GAZEBO_CONVERSIONS_HH_
#define IGNITION_GAZEBO_CONVERSIONS_HH_

#include <ignition/msgs/axis.pb.h>
#include <ignition/msgs/geometry.pb.h>
#include <ignition/msgs/gui.pb.h>
#include <ignition/msgs/inertial.pb.h>
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/material.pb.h>
#include <ignition/msgs/scene.pb.h>
#include <ignition/msgs/sensor.pb.h>
#include <ignition/msgs/sensor_noise.pb.h>
#include <ignition/msgs/time.pb.h>
#include <ignition/msgs/world_stats.pb.h>

#include <chrono>

#include <ignition/common/Console.hh>
#include <ignition/math/Inertial.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Gui.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Light.hh>
#include <sdf/Material.hh>
#include <sdf/Noise.hh>
#include <sdf/Scene.hh>
#include <sdf/Sensor.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    /// \brief Helper function that sets a mutable msgs::SensorNoise object
    /// to the values contained in a sdf::Noise object.
    /// \param[out] _msg SensorNoise message to set.
    /// \param[in] _sdf SDF Noise object.
    void set(msgs::SensorNoise *_msg, const sdf::Noise &_sdf);

    /// \brief Generic conversion from an SDF geometry to another type.
    /// \param[in] _in SDF geometry.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Geometry &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF geometry to a geometry
    /// message.
    /// \param[in] _in SDF geometry.
    /// \return Geometry message.
    template<>
    msgs::Geometry convert(const sdf::Geometry &_in);

    /// \brief Generic conversion from a geometry message to another type.
    /// \param[in] _in Geometry message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Geometry &/*_in*/)
    {
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
    Out convert(const sdf::Material &/*_in*/)
    {
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
    Out convert(const msgs::Material &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a material message to a material
    /// SDF object.
    /// \param[in] _in Material message.
    /// \return SDF material.
    template<>
    sdf::Material convert(const msgs::Material &_in);

    /// \brief Generic conversion from an SDF light to another type.
    /// \param[in] _in SDF light.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Light &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF light to a light
    /// message.
    /// \param[in] _in SDF light.
    /// \return Light message.
    template<>
    msgs::Light convert(const sdf::Light &_in);


    /// \brief Generic conversion from a light message to another type.
    /// \param[in] _in Light message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Light& /*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a light message to a light
    /// SDF object.
    /// \param[in] _in Light message.
    /// \return Light SDF object.
    template<>
    sdf::Light convert(const msgs::Light &_in);

    /// \brief Generic conversion from an SDF gui to another type.
    /// \param[in] _in SDF gui.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Gui &/*_in*/)
    {
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
    Out convert(const std::chrono::steady_clock::duration &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a steady clock duration to a time
    /// message.
    /// \param[in] _in Steady clock duration.
    /// \return Ignition time message.
    template<>
    msgs::Time convert(const std::chrono::steady_clock::duration &_in);

    /// \brief Generic conversion from a time message to another type.
    /// \param[in] _in Time message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Time &/*_in*/)
    {
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
    Out convert(const math::Inertiald &/*_in*/)
    {
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
    Out convert(const msgs::Inertial &/*_in*/)
    {
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
    Out convert(const sdf::JointAxis &/*_in*/)
    {
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
    Out convert(const msgs::Axis &/*_in*/)
    {
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
    Out convert(const sdf::Scene &/*_in*/)
    {
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
    Out convert(const msgs::Scene &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a scene message to a scene
    /// SDF object.
    /// \param[in] _in Scene message.
    /// \return SDF scene.
    template<>
    sdf::Scene convert(const msgs::Scene &_in);

    /// \brief Generic conversion from an SDF Sensor to another type.
    /// \param[in] _in SDF Sensor.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const sdf::Sensor &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF sensor to a sensor
    /// message.
    /// \param[in] _in SDF geometry.
    /// \return Sensor message.
    template<>
    msgs::Sensor convert(const sdf::Sensor &_in);

    /// \brief Generic conversion from a sensor message to another type.
    /// \param[in] _in Sensor message.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const msgs::Sensor &/*_in*/)
    {
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
    Out convert(const msgs::SensorNoise &/*_in*/)
    {
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
    Out convert(const msgs::WorldStatistics &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a world statistics message to an
    /// `ignition::gazebo::UpdateInfo` object.
    /// \param[in] _in WorldStatistics message.
    /// \return Update info.
    template<>
    UpdateInfo convert(const msgs::WorldStatistics &_in);

    /// \brief Generic conversion from update info to another type.
    /// \param[in] _in Update info.
    /// \return Conversion result.
    /// \tparam Out Output type.
    template<class Out>
    Out convert(const UpdateInfo &/*_in*/)
    {
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
    Out convert(const sdf::Collision &/*_in*/)
    {
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
    Out convert(const msgs::Collision &/*_in*/)
    {
      Out::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from a collision message to a collision
    /// SDF object.
    /// \param[in] _in Collision message.
    /// \return SDF collision.
    template<>
    sdf::Collision convert(const msgs::Collision &_in);
    }
  }
}
#endif
