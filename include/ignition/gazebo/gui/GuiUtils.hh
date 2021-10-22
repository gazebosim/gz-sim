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
#ifndef IGNITION_GAZEBO_GUI_GUIUTILS_HH_
#define IGNITION_GAZEBO_GUI_GUIUTILS_HH_

#include <string>

namespace ignition
{
namespace gazebo
{
namespace gui
{

/// \brief Enumeration of available primitive shape types
enum class PrimitiveShape
{
  kBox,
  kSphere,
  kCylinder,
  kCapsule,
  kEllipsoid,
};

/// \brief Enumeration of available primitive light types
enum class PrimitiveLight
{
  kDirectional,
  kPoint,
  kSpot,
};

/// \brief Return an SDF string of one of the avilable primitive shape types
/// \param[in] _type Type of shape to retrieve
/// \return String containing SDF description of primitive shape
std::string getPrimitiveShape(const PrimitiveShape &_type);

/// \brief Return an SDF string of one of the avilable primitive light types
/// \param[in] _type Type of light to retrieve
/// \return String containing SDF description of primitive light
std::string getPrimitiveLight(const PrimitiveLight &_type);

}  // namespace gui
}  // namespace gazebo
}  // namespace ignition
#endif
