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

#include <ignition/common/Console.hh>
#include <ignition/msgs/boxgeom.pb.h>
#include <ignition/msgs/cylindergeom.pb.h>
#include <ignition/msgs/geometry.pb.h>
#include <ignition/msgs/spheregeom.pb.h>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Sphere.hh>
#include <sdf/Geometry.hh>

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {

    /// \brief Generic conversion from an SDF geometry to another type.
    /// \param[in] _in SDF geometry
    /// \return Conversion result
    /// \tparam OUT Output type
    template<class OUT>
    OUT Convert(const sdf::Geometry &_in)
    {
      OUT::ConversionNotImplemented;
    }

    /// \brief Specialized conversion from an SDF geometry to a geometry
    /// message.
    /// \param[in] _in SDF geometry.
    /// \return Geometry message.
    template<>
    msgs::Geometry Convert(const sdf::Geometry &_in)
    {
      msgs::Geometry out;
      if (_in.Type() == sdf::GeometryType::BOX && _in.BoxShape())
      {
        out.set_type(msgs::Geometry::BOX);
        msgs::Set(out.mutable_box()->mutable_size(), _in.BoxShape()->Size());
      }
      else if (_in.Type() == sdf::GeometryType::CYLINDER && _in.CylinderShape())
      {
        out.set_type(msgs::Geometry::CYLINDER);
        out.mutable_cylinder()->set_radius(_in.CylinderShape()->Radius());
        out.mutable_cylinder()->set_length(_in.CylinderShape()->Length());
      }
      else if (_in.Type() == sdf::GeometryType::SPHERE && _in.SphereShape())
      {
        out.set_type(msgs::Geometry::SPHERE);
        out.mutable_sphere()->set_radius(_in.SphereShape()->Radius());
      }
      else
      {
        ignerr << "Geometry type [" << static_cast<int>(_in.Type())
               << "] not supported" << std::endl;
      }
      return out;
    }
    }
  }
}
#endif
