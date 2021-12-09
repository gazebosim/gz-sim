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
#ifndef IGNITION_GAZEBO_PARAMETERS_HH_
#define IGNITION_GAZEBO_PARAMETERS_HH_

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/ParameterDeclarationCmd.hh>
#include <ignition/gazebo/components/ParametersRegistry.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Declare a parameter
  template<typename ComponentT>
  bool
  DeclareParameter(EntityComponentManager &_ecm,const std::string & parameter_name, typename ComponentT::Type & data)
  {
    auto worldEntity = _ecm.EntityByComponents(components::World());
    if (kNullEntity == worldEntity) {
      return false;
    }
    std::ostringstream ostr;
    data->Serialize(ostr);
    auto * registry = _ecm.Component<components::ParameterDeclarationCmd>(worldEntity);
    if (!registry) {
      msgs::ParameterDeclarations declarations;
      auto * declaration = declarations.add_parameter_declaration();
      declaration->set_type_name(ComponentT::Type::typeName);
      declaration->set_name(parameter_name);
      declaration->set_value(ostr.str());
      _ecm.CreateComponent<components::ParameterDeclarationCmd>(declarations);
      return true;
    }
    auto & declarations = registry->Data();
    auto * declaration = declarations.add_parameter_declaration();
    declaration->set_type_name(ComponentT::Type::typeName);
    declaration->set_name(parameter_name);
    declaration->set_value(ostr.str());
    return true;
  }
}
}
}

#endif
