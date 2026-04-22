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
 */

#ifndef GZ_SIM_PYTHON_ENTITYITERATION_HH_
#define GZ_SIM_PYTHON_ENTITYITERATION_HH_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/detail/ComponentPybindRegistry.hh>

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{



class ECMPythonAccessor
{
  public: static const gz::sim::components::BaseComponent *Component(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity, gz::sim::ComponentTypeId _typeId)
  {
    return _ecm.Component(_entity, _typeId);
  }

  public: static gz::sim::components::BaseComponent *Component(
      gz::sim::EntityComponentManager &_ecm, const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId)
  {
    return _ecm.Component(_entity, _typeId);
  }



  public: static pybind11::list EachList(
      gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::sim::ComponentTypeId> &_types)
  {
    pybind11::list result;
    auto registry = gz::sim::python::ComponentPybindRegistry::Instance();

    auto callback =
        [&](Entity entity,
            const std::vector<const components::BaseComponent *> &compData)
    {
      pybind11::list py_components;
      for (size_t i = 0; i < _types.size(); ++i)
      {
        auto compBase = compData[i];
        if (compBase)
        {
          auto typeId = _types[i];
          auto getter = registry->Getter(typeId);
          if (getter)
          {
            py_components.append(getter(_ecm, entity));
          }
          else
          {
            py_components.append(pybind11::none());
          }
        }
        else
        {
          py_components.append(pybind11::none());
        }
      }
      result.append(
          pybind11::make_tuple(pybind11::cast(entity), py_components));
    };

    _ecm.EntitiesByComponentIds(_types, callback);

    return result;
  }
};

}  // namespace python
}  // namespace sim
}  // namespace gz

#endif
