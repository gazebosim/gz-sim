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

%module world
%{
#include <ignition/gazebo/World.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/math/Vector3.hh>
%}

%include "stdint.i"
%include <std_string.i>

/*
%include "typemaps.i"
%typemap(out) (std::optional< ignition::math::Vector3< double > >) %{
  if((*(&result)).has_value()) {
    $result = SWIG_NewPointerObj(
      (new ignition::math::Vector3< double >(static_cast< const ignition::math::Vector3< double >& >((*(&result)).value()))),
      SWIGTYPE_p_ignition__math__Vector3T_double_t,
      SWIG_POINTER_OWN |  0 );
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}
*/

namespace ignition
{
  namespace gazebo
  {
    class World
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";

      public: World(ignition::gazebo::Entity _entity = kNullEntity);
      public: ~World();
      public: uint64_t ModelByName(const ignition::gazebo::EntityComponentManager &_ecm,
          const std::string &_name) const;

      public: std::optional< ignition::math::Vector3< double> > Gravity(
          const ignition::gazebo::EntityComponentManager &_ecm) const;
    };
  }
}
