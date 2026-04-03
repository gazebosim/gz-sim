/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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
#ifndef GZ_SIM_DETAIL_COMPONENTPYBINDREGISTRY_HH_
#define GZ_SIM_DETAIL_COMPONENTPYBINDREGISTRY_HH_

// Handle Qt's "slots" macro
#ifdef slots
#pragma push_macro("slots")
#undef slots
#include <pybind11/pybind11.h>
#pragma pop_macro("slots")
#else
#include <pybind11/pybind11.h>
#endif

#include <functional>
#include <unordered_map>
#include <iostream>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/utils/NeverDestroyed.hh>

namespace gz
{
namespace sim
{
namespace python
{
/// \brief A class that holds functions for getting and setting component
/// data for use with pybind11.
class ComponentPybindRegistry
{
  public: static ComponentPybindRegistry *Instance()
  {
    static gz::utils::NeverDestroyed<ComponentPybindRegistry> instance;
    return &instance.Access();
  }
  /// \brief Function for getting component data as a python object.
  public: using GetterFn = std::function<pybind11::object(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity)>;

  /// \brief Function for setting component data from a python object.
  public:
  using SetterFn = std::function<void(gz::sim::EntityComponentManager &_ecm,
                                      const gz::sim::Entity &_entity,
                                      const pybind11::object &_obj)>;

  /// \brief Add a getter function for a component type.
  /// \param[in] _typeId The component type ID.
  /// \param[in] _fn The getter function.
  public: void AddGetter(ComponentTypeId _typeId, GetterFn _fn)
  {
    // std::cout << "Adding getter for: " << _typeId << " " << this->getters.size()
    //           << std::endl;
    this->getters[_typeId] = _fn;
  }

  /// \brief Add a setter function for a component type.
  /// \param[in] _typeId The component type ID.
  /// \param[in] _fn The setter function.
  public: void AddSetter(ComponentTypeId _typeId, SetterFn _fn)
  {
    this->setters[_typeId] = _fn;
  }

  /// \brief Get a getter function for a component type.
  /// \param[in] _typeId The component type ID.
  /// \return The getter function, or nullptr if not found.
  public: GetterFn Getter(ComponentTypeId _typeId) const
  {
    std::cout << "Looking for getter for: " << _typeId << std::endl;
    auto it = this->getters.find(_typeId);
    if (it == this->getters.end())
      return nullptr;
    return it->second;
  }

  /// \brief Get a setter function for a component type.
  /// \param[in] _typeId The component type ID.
  /// \return The setter function, or nullptr if not found.
  public: SetterFn Setter(ComponentTypeId _typeId) const
  {
    auto it = this->setters.find(_typeId);
    if (it == this->setters.end())
      return nullptr;
    return it->second;
  }

  private: std::unordered_map<ComponentTypeId, GetterFn> getters;

  private: std::unordered_map<ComponentTypeId, SetterFn> setters;
};

template <typename T>
struct AddPybindGetterSetter
{
  static pybind11::object Getter(const gz::sim::EntityComponentManager &_ecm,
                                 const gz::sim::Entity &_entity)
  {
    auto comp = _ecm.Component<T>(_entity);
    // std::cout << "Getting " << T::typeName << " id: " << T::typeId
    //           << " on entity " << _entity << std::endl;
    if (comp)
    {
      return pybind11::cast(comp->Data());
    }
    return pybind11::none();
  }

  static void Setter(gz::sim::EntityComponentManager &_ecm,
                     const gz::sim::Entity &_entity,
                     const pybind11::object &_obj)
  {
    if (_obj.is_none())
    {
      return;
    }
    try
    {
      auto data = pybind11::cast<typename T::Type>(_obj);
      T *comp = _ecm.Component<T>(_entity);

      if (nullptr == comp)
      {
        _ecm.CreateComponent(_entity, T(data));
        return;
      }
      // std::cout << "Setting " << T::typeName << " id: " << T::typeId
      //           << " on entity " << _entity << std::endl;
      comp->Data() = data;
    }
    catch (const pybind11::cast_error &e)
    {
      std::cerr << "Failed to cast python object to component data type: "
                << e.what() << std::endl;
    }
  }
  AddPybindGetterSetter()
  {
    ComponentPybindRegistry::Instance()->AddGetter(T::typeId, Getter);
    ComponentPybindRegistry::Instance()->AddSetter(T::typeId, Setter);
  }
};

template <typename U>
struct AddPybindGetterSetter<
    gz::sim::components::Component<gz::sim::components::NoData, U>>
{
  AddPybindGetterSetter() = default;
};

}  // namespace python
}  // namespace sim
}  // namespace gz

#endif
