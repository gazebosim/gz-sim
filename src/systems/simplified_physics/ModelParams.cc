/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#include "ModelParams.hh"

#include <cmath>
#include <cstddef>
#include <exception>
#include <string>

#include <gz/common/Console.hh>

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::simplified_physics
{
namespace
{
/// \brief Read a whole string as a finite number.
/// \param[in] _text Text, with optional spaces around the number.
/// \param[out] _value The number, only set on success.
/// \return True if the text is a finite number and nothing else.
bool ReadNumber(const std::string &_text, double &_value)
{
  try
  {
    std::size_t used{0};
    const double value = std::stod(_text, &used);
    if (!std::isfinite(value) ||
        _text.find_first_not_of(" \t\r\n", used) != std::string::npos)
    {
      return false;
    }
    _value = value;
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}
}

//////////////////////////////////////////////////
std::vector<sdf::ElementPtr> PluginContents(const sdf::Element &_plugin)
{
  std::vector<sdf::ElementPtr> contents;
  for (auto child = _plugin.GetFirstElement(); child;
       child = child->GetNextElement())
  {
    contents.push_back(child);
  }
  return contents;
}

//////////////////////////////////////////////////
ModelParams ParseModelParams(const std::vector<sdf::ElementPtr> &_contents,
    const std::string &_model, bool _warn)
{
  ModelParams params;

  for (const auto &element : _contents)
  {
    if (!element)
      continue;

    const auto &name = element->GetName();
    const auto text = element->Get<std::string>("", "").first;

    if (name == "odometry_noise")
    {
      double value{0.0};
      if (ReadNumber(text, value) && value >= 0.0)
      {
        params.odometryNoise = value;
      }
      else if (_warn)
      {
        gzwarn << "Model [" << _model << "] has an invalid odometry_noise ["
               << text << "]. It must be a number of at least 0. The noise "
               << "is off." << std::endl;
      }
    }
    else if (name == "animate_wheels")
    {
      params.animateWheels = text == "true" || text == "1";
      if (params.animateWheels && _warn)
      {
        gzwarn << "Model [" << _model << "] asks for animate_wheels, which "
               << "is not implemented yet. The wheel links do not turn."
               << std::endl;
      }
    }
    else if (name == "collision_response")
    {
      if (text == "stop")
      {
        params.collisionResponse = PlanarResponse::Stop;
      }
      else if (text == "slide")
      {
        params.collisionResponse = PlanarResponse::Slide;
      }
      else if (_warn)
      {
        gzwarn << "Model [" << _model << "] has an unknown "
               << "collision_response [" << text << "]. Known values are "
               << "[stop] and [slide]. The model uses [slide]." << std::endl;
      }
    }
    else if (name == "wheel")
    {
      WheelOverride entry;
      entry.joint = text;
      if (entry.joint.empty())
      {
        if (_warn)
        {
          gzwarn << "Model [" << _model << "] has a <wheel> element without "
                 << "a joint name. It is ignored." << std::endl;
        }
        continue;
      }

      if (element->HasAttribute("radius"))
      {
        double radius{0.0};
        const auto radiusText =
            element->Get<std::string>("radius", "").first;
        if (ReadNumber(radiusText, radius) && radius > 0.0)
        {
          entry.radius = radius;
        }
        else if (_warn)
        {
          gzwarn << "Model [" << _model << "] has an invalid radius ["
                 << radiusText << "] on wheel [" << entry.joint << "]. The "
                 << "radius comes from its collision." << std::endl;
        }
      }
      params.wheels.push_back(entry);
    }
  }
  return params;
}
}
}
}
