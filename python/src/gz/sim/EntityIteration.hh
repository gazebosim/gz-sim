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

#include <gz/sim/entt/entity/runtime_view.hpp>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/python/ComponentPybindRegistry.hh>

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{

class ECMPythonAccessor
{
  private: template <typename TagType = void>
  static pybind11::list EachListImpl(
      gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::sim::ComponentTypeId> &_types)
  {
    pybind11::list result;
    auto registry = gz::sim::python::ComponentPybindRegistry::Instance();

    using SparseSetType =
        std::remove_pointer_t<decltype(_ecm.Registry().storage(0))>;
    entt::basic_runtime_view<SparseSetType, std::allocator<SparseSetType *>> view;

    if constexpr (!std::is_same_v<TagType, void>)
    {
      auto &tagStorage = _ecm.Registry().storage<TagType>();
      view.iterate(tagStorage);
    }

    std::vector<SparseSetType *> storages;
    storages.reserve(_types.size());

    for (auto typeId : _types)
    {
      auto *storage = _ecm.Registry().storage(typeId);
      if (!storage)
        return result;
      view.iterate(*storage);
      storages.push_back(storage);
    }

    for (auto entity : view)
    {
      if constexpr (std::is_same_v<TagType, void>)
      {
        if (_ecm.IsMarkedForRemoval(entity))
          continue;
      }

      pybind11::list py_components;
      for (size_t i = 0; i < _types.size(); ++i)
      {
        auto *storage = storages[i];
        auto compBase = static_cast<const components::BaseComponent *>(storage->value(entity));
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
    }

    return result;
  }

  public: static pybind11::list EachList(
      gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::sim::ComponentTypeId> &_types)
  {
    return EachListImpl<void>(_ecm, _types);
  }

  public: static pybind11::list EachNewList(
      gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::sim::ComponentTypeId> &_types)
  {
    return EachListImpl<gz::sim::NewEntity>(_ecm, _types);
  }

  public: static pybind11::list EachRemovedList(
      gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::sim::ComponentTypeId> &_types)
  {
    return EachListImpl<gz::sim::RemoveEntity>(_ecm, _types);
  }
};

}  // namespace python
}  // namespace sim
}  // namespace gz

#endif
