/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENTS_FACTORY_HH_
#define IGNITION_GAZEBO_COMPONENTS_FACTORY_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <ignition/common/SingletonT.hh>
#include <ignition/common/Util.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A factory that generates a component based on a string type.
  class IGNITION_GAZEBO_VISIBLE Factory
      : public ignition::common::SingletonT<Factory>
  {
    /// \brief Register a component.
    /// \param[in] _type Type of component to register.
    public: template<typename ComponentTypeT>
    void Register(const std::string &_type)
    {
      std::function<std::unique_ptr<ComponentTypeT>()> f = []()
      {
        return std::make_unique<ComponentTypeT>();
      };

      auto id = EntityComponentManager::ComponentType<ComponentTypeT>();
      compsByName[_type] = f;
      compsById[id] = f;

      // Initialize static member variables.
      // ComponentTypeT::name = _type;
      // ComponentTypeT::id = ignition::common::hash64(_type);
    }

    /// \brief Create a new instance of a component.
    /// \param[in] _type Component key to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: template<typename T, typename KeyT>
    std::unique_ptr<T> New(const KeyT &_type)
    {
      return std::unique_ptr<T>(static_cast<T*>(New(_type).release()));
    }

    /// \brief Create a new instance of a component.
    /// \param[in] _type Type of component to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: std::unique_ptr<components::Component> New(
        const std::string &_type)
    {
      std::string type;
      const std::string kCompStr  = "ign_gazebo_components.";
      const std::string kCompStr1 = "ignition.gazebo.components.";
      const std::string kCompStr2 = ".ignition.gazebo.components.";
      // Convert "ignition.gazebo.components." to "ign_gazebo_components.".
      if (_type.compare(0, kCompStr1.size(), kCompStr1) == 0)
      {
        type = kCompStr + _type.substr(kCompStr1.size());
      }
      // Convert ".ignition.gazebo.components" to "ign_gazebo_components.".
      else if (_type.compare(0, kCompStr2.size(), kCompStr2) == 0)
      {
        type = kCompStr + _type.substr(kCompStr2.size());
      }
      else
      {
        // Fix typenames that are missing "ign_gazebo_components."
        // at the beginning.
        if (_type.compare(0, kCompStr.size(), kCompStr) != 0)
          type = kCompStr;
        type += _type;
      }

      // Create a new component if a FactoryFn has been assigned to this type.
      std::unique_ptr<components::Component> comp;
      auto it = compsByName.find(type);
      if (it != compsByName.end())
        comp = it->second();

      return comp;
}

    /// \brief Create a new instance of a component.
    /// \param[in] _type Component id to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: std::unique_ptr<components::Component> New(
        const ComponentTypeId &_type)
    {
      // Create a new component if a FactoryFn has been assigned to this type.
      std::unique_ptr<components::Component> comp;
      auto it = compsById.find(_type);
      if (it != compsById.end())
        comp = it->second();

      return comp;
    }

    /// \brief Get all the component types.
    /// return Vector of strings of the component types.
    public: std::vector<std::string> Components()
    {
      std::vector<std::string> types;

      // Return the list of all known component types.
      for (const auto & [name, funct] : compsByName)
        types.push_back(name);

      return types;
    }

    /// \typedef FactoryFn
    /// \brief Prototype for component factory generation.
    private: using FactoryFn =
        std::function<std::unique_ptr<components::Component>()>;

    /// \brief A list of registered components where the key is its name.
    private: std::map<std::string, FactoryFn> compsByName;

    /// \brief A list of registered components where the key is its id.
    private: std::map<ComponentTypeId, FactoryFn> compsById;
  };

  /// \brief Static component registration macro.
  ///
  /// Use this macro to register components.
  /// \param[in] _compType Component type name.
  /// \param[in] _classname Class name for component.
  #define IGN_GAZEBO_REGISTER_COMPONENT(_compType, _classname) \
  class IGNITION_GAZEBO_VISIBLE IgnGazeboComponents##_classname \
  { \
    public: IgnGazeboComponents##_classname() \
    { \
      ignition::gazebo::components::Factory::Instance()->Register<_classname>(\
        _compType);\
    } \
  }; \
  static IgnGazeboComponents##_classname\
    IgnitionGazeboComponentsInitializer##_classname;
}
}
}
}

#endif
