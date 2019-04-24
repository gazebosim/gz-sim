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
#include <ignition/msgs/time.pb.h>

#include <chrono>

#include <ignition/common/Console.hh>
#include <ignition/math/Inertial.hh>
#include <sdf/Geometry.hh>
#include <sdf/Gui.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Light.hh>
#include <sdf/Material.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Export.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
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
    }
  }
}
#endif
