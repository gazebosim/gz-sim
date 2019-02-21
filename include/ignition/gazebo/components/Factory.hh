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
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A base class for an object responsible for creating components.
  class IGNITION_GAZEBO_VISIBLE ComponentDescriptorBase
  {
    /// \brief Destructor
    public: virtual ~ComponentDescriptorBase() = default;

    /// \brief Create an instance of a Component.
    /// \return Pointer to a component.
    public: virtual std::unique_ptr<components::BaseComponent> Create() const
        = 0;
  };

  /// \brief A class for an object responsible for creating components.
  /// \tparam ComponentTypeT type of component to describe.
  template <typename ComponentTypeT>
  class IGNITION_GAZEBO_VISIBLE ComponentDescriptor
    : public ComponentDescriptorBase
  {
    /// \brief Create an instance of a ComponentTypeT Component.
    /// \return Pointer to a component.
    public: std::unique_ptr<components::BaseComponent> Create() const override
    {
      return std::make_unique<ComponentTypeT>();
    }
  };

  /// \brief A factory that generates a component based on a string type.
  class IGNITION_GAZEBO_VISIBLE Factory
      : public ignition::common::SingletonT<Factory>
  {
    /// \brief Register a component.
    /// \param[in] _type Type of component to register.
    /// \param[in] _desc Object to manage the creation of ComponentTypeT
    ///   objects.
    /// \tparam ComponentTypeT Type of component to register.
    public: template<typename ComponentTypeT>
    void Register(const std::string &_type, ComponentDescriptorBase *_desc)
    {
      // Initialize static member variables.
      ComponentTypeT::typeName = _type;
      ComponentTypeT::typeId = ignition::common::hash64(_type);

      this->compsByName[ComponentTypeT::typeName] = _desc;
      this->compsById[ComponentTypeT::typeId] = _desc;
    }

    /// \brief Create a new instance of a component.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    /// \tparam ComponentTypeT component type requested
    public: template<typename ComponentTypeT>
    std::unique_ptr<ComponentTypeT> New()
    {
      return std::unique_ptr<ComponentTypeT>(static_cast<ComponentTypeT*>(
            New(ComponentTypeT::typeName).release()));
    }

    /// \brief Create a new instance of a component.
    /// \param[in] _type Type of component to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: std::unique_ptr<components::BaseComponent> New(
        const std::string &_type)
    {
      std::string type;
      // Convert "ignition.gazebo.components." to "ign_gazebo_components.".
      if (_type.compare(0, strlen(kCompStr1), kCompStr1) == 0)
      {
        type = kCompStr + _type.substr(strlen(kCompStr1));
      }
      // Convert ".ignition.gazebo.components" to "ign_gazebo_components.".
      else if (_type.compare(0, strlen(kCompStr2), kCompStr2) == 0)
      {
        type = kCompStr + _type.substr(strlen(kCompStr2));
      }
      else
      {
        // Fix typenames that are missing "ign_gazebo_components."
        // at the beginning.
        if (_type.compare(0, strlen(kCompStr), kCompStr) != 0)
          type = kCompStr;
        type += _type;
      }

      // Create a new component if a Descriptor has been assigned to this type.
      std::unique_ptr<components::BaseComponent> comp;
      auto it = this->compsByName.find(type);
      if (it != this->compsByName.end())
        comp = it->second->Create();

      return comp;
    }

    /// \brief Create a new instance of a component.
    /// \param[in] _type Component id to create.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: std::unique_ptr<components::BaseComponent> New(
        const ComponentTypeId &_type)
    {
      // Create a new component if a FactoryFn has been assigned to this type.
      std::unique_ptr<components::BaseComponent> comp;
      auto it = this->compsById.find(_type);
      if (it != this->compsById.end())
        comp = it->second->Create();

      return comp;
    }

    /// \brief Get all the registered component types by type name.
    /// return Vector of strings with the component type names.
    public: std::vector<std::string> TypeNames() const
    {
      std::vector<std::string> types;

      // Return the list of all known component types.
      for (const auto &[name, funct] : this->compsByName)
        types.push_back(name);

      return types;
    }

    /// \brief Get all the registered component types by ID.
    /// return Vector of component IDs.
    public: std::vector<uint64_t> TypeIds() const
    {
      std::vector<ComponentTypeId> types;

      // Return the list of all known component types.
      for (const auto &[id, funct] : this->compsById)
        types.push_back(id);

      return types;
    }

    /// \brief A list of registered components where the key is its name.
    ///
    /// Note about compsByName and compsById. The maps store pointers as the
    /// values, but never cleans them up, which may (at first glance) seem like
    /// incorrect behavior. This is not a mistake. Since ComponentDescriptors
    /// are created at the point in the code where components are defined, this
    /// generally ends up in a shared library that will be loaded at runtime.
    ///
    /// Because this and the plugin loader both use static variables, and the
    /// order of static initialization and destruction are not guaranteed, this
    /// can lead to a scenario where the shared library is unloaded (with the
    /// ComponentDescriptor), but the Factory still exists. For this reason,
    /// we just keep a pointer, which will dangle until the program is shutdown.
    private: std::map<std::string, ComponentDescriptorBase *> compsByName;

    /// \brief A list of registered components where the key is its id.
    private: std::map<ComponentTypeId, ComponentDescriptorBase *> compsById;

    /// \brief Valid component name prefix
    private: constexpr static const char *kCompStr {
               "ign_gazebo_components."};

    /// \brief Component name prefix that should be changed
    private: constexpr static const char *kCompStr1 {
               "ignition.gazebo.components."};

    /// \brief Component name prefix that should be changed
    private: constexpr static const char *kCompStr2 {
               ".ignition.gazebo.components."};
  };

  /// \brief Static component registration macro.
  ///
  /// Use this macro to register components.
  /// \param[in] _compType Component type name.
  /// \param[in] _classname Class name for component.
  #define IGN_GAZEBO_REGISTER_COMPONENT(_compType, _classname) \
  inline IGNITION_GAZEBO_VISIBLE \
  std::unique_ptr<ignition::gazebo::components::BaseComponent> \
      New##_classname() \
  { \
    return std::unique_ptr<ignition::gazebo::components::_classname>(\
        new ignition::gazebo::components::_classname); \
  } \
  class IGNITION_GAZEBO_VISIBLE IgnGazeboComponents##_classname \
  { \
    public: IgnGazeboComponents##_classname() \
    { \
      using namespace ignition;\
      using Desc = gazebo::components::ComponentDescriptor<_classname>; \
      gazebo::components::Factory::Instance()->Register<_classname>(\
        _compType, new Desc());\
    } \
  }; \
  static IgnGazeboComponents##_classname\
    IgnitionGazeboComponentsInitializer##_classname;
}
}
}
}

#endif
