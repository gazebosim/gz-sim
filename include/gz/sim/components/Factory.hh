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
#ifndef GZ_SIM_COMPONENTS_FACTORY_HH_
#define GZ_SIM_COMPONENTS_FACTORY_HH_

#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/SingletonT.hh>
#include <gz/common/Util.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/detail/ComponentStorageBase.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Types.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A base class for an object responsible for creating components.
  class ComponentDescriptorBase
  {
    /// \brief Destructor
    public: virtual ~ComponentDescriptorBase() = default;

    /// \brief Create an instance of a Component.
    /// \return Pointer to a component.
    public: virtual std::unique_ptr<BaseComponent> Create() const = 0;

    /// \brief Create an instance of a Component, populated with specific data.
    /// \param[in] _data The data to populate the component with.
    /// \return Pointer to a component.
    public: virtual std::unique_ptr<BaseComponent> Create(
                const components::BaseComponent *_data) const = 0;
  };

  /// \brief A class for an object responsible for creating components.
  /// \tparam ComponentTypeT type of component to describe.
  template <typename ComponentTypeT>
  class ComponentDescriptor
    : public ComponentDescriptorBase
  {
    /// \brief Documentation inherited
    public: std::unique_ptr<BaseComponent> Create() const override
    {
      return std::make_unique<ComponentTypeT>();
    }

    /// \brief Documentation inherited
    public: virtual std::unique_ptr<BaseComponent> Create(
                const components::BaseComponent *_data) const override
    {
      ComponentTypeT comp(*static_cast<const ComponentTypeT *>(_data));
      return std::make_unique<ComponentTypeT>(comp);
    }
  };

  /// \brief A base class for an object responsible for creating storages.
  class StorageDescriptorBase
  {
    /// \brief Constructor
    public: IGN_DEPRECATED(6) StorageDescriptorBase() = default;

    /// \brief Destructor
    public: virtual ~StorageDescriptorBase() = default;

    /// \brief Create an instance of a storage.
    /// \return Pointer to a storage.
    public: virtual std::unique_ptr<ComponentStorageBase> Create() const  = 0;
  };

  /// \brief A class for an object responsible for creating storages.
  /// \tparam ComponentTypeT type of component that the storage will hold.
  template <typename ComponentTypeT>
  class StorageDescriptor
    : public StorageDescriptorBase
  {
    /// \brief Constructor
    public: IGN_DEPRECATED(6) StorageDescriptor() = default;

    /// \brief Create an instance of a storage that holds ComponentTypeT
    /// components.
    /// \return Pointer to a component.
    public: std::unique_ptr<ComponentStorageBase> Create() const override
    {
      return std::make_unique<ComponentStorage<ComponentTypeT>>();
    }
  };

  /// \brief A factory that generates a component based on a string type.
  class Factory
      : public gz::common::SingletonT<Factory>
  {
    /// \brief Register a component so that the factory can create instances
    /// of the component and its storage based on an ID.
    /// \param[in] _type Type of component to register.
    /// \param[in] _compDesc Object to manage the creation of ComponentTypeT
    ///  objects.
    /// \param[in] _storageDesc Ignored.
    /// \tparam ComponentTypeT Type of component to register.
    /// \deprecated See function that doesn't accept a storage
    public: template<typename ComponentTypeT>
    void IGN_DEPRECATED(6) Register(const std::string &_type,
        ComponentDescriptorBase *_compDesc,
        StorageDescriptorBase * /*_storageDesc*/)
    {
      this->Register<ComponentTypeT>(_type, _compDesc);
    }

    /// \brief Register a component so that the factory can create instances
    /// of the component based on an ID.
    /// \param[in] _type Type of component to register.
    /// \param[in] _compDesc Object to manage the creation of ComponentTypeT
    ///  objects.
    /// \tparam ComponentTypeT Type of component to register.
    public: template<typename ComponentTypeT>
    void Register(const std::string &_type, ComponentDescriptorBase *_compDesc)
    {
      // Every time a plugin which uses a component type is loaded, it attempts
      // to register it again, so we skip it.
      if (ComponentTypeT::typeId != 0)
      {
        return;
      }

      auto typeHash = gz::common::hash64(_type);

      // Initialize static member variable - we need to set these
      // static members for every shared lib that uses the component, but we
      // only add them to the maps below once.
      ComponentTypeT::typeId = typeHash;
      ComponentTypeT::typeName = _type;

      // Check if component has already been registered by another library
      auto runtimeName = typeid(ComponentTypeT).name();
      auto runtimeNameIt = this->runtimeNamesById.find(typeHash);
      if (runtimeNameIt != this->runtimeNamesById.end())
      {
        // Warn user if type was previously registered with a different name.
        // We're leaving the ID set in case this is a false difference across
        // libraries.
        if (runtimeNameIt->second != runtimeName)
        {
          std::cerr
            << "Registered components of different types with same name: type ["
            << runtimeNameIt->second << "] and type [" << runtimeName
            << "] with name [" << _type << "]. Second type will not work."
            << std::endl;
        }
        return;
      }

      // This happens at static initialization time, so we can't use common
      // console
      std::string debugEnv;
      gz::common::env("IGN_DEBUG_COMPONENT_FACTORY", debugEnv);
      if (debugEnv == "true")
      {
        std::cout << "Registering [" << ComponentTypeT::typeName << "]"
                  << std::endl;
      }

      // Keep track of all types
      this->compsById[ComponentTypeT::typeId] = _compDesc;
      namesById[ComponentTypeT::typeId] = ComponentTypeT::typeName;
      runtimeNamesById[ComponentTypeT::typeId] = runtimeName;
    }

    /// \brief Unregister a component so that the factory can't create instances
    /// of the component anymore.
    /// \tparam ComponentTypeT Type of component to unregister.
    public: template<typename ComponentTypeT>
    void Unregister()
    {
      this->Unregister(ComponentTypeT::typeId);

      ComponentTypeT::typeId = 0;
    }

    /// \brief Unregister a component so that the factory can't create instances
    /// of the component anymore.
    /// \details This function will not reset the `typeId` static variable
    /// within the component type itself. Prefer using the templated
    /// `Unregister` function when possible.
    /// \param[in] _typeId Type of component to unregister.
    public: void Unregister(ComponentTypeId _typeId)
    {
      // Not registered
      if (_typeId == 0)
      {
        return;
      }

      {
        auto it = this->compsById.find(_typeId);
        if (it != this->compsById.end())
        {
          delete it->second;
          this->compsById.erase(it);
        }
      }

      {
        auto it = namesById.find(_typeId);
        if (it != namesById.end())
        {
          namesById.erase(it);
        }
      }

      {
        auto it = runtimeNamesById.find(_typeId);
        if (it != runtimeNamesById.end())
        {
          runtimeNamesById.erase(it);
        }
      }
    }

    /// \brief Create a new instance of a component.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    /// \tparam ComponentTypeT component type requested
    public: template<typename ComponentTypeT>
    std::unique_ptr<ComponentTypeT> New()
    {
      return std::unique_ptr<ComponentTypeT>(static_cast<ComponentTypeT *>(
            this->New(ComponentTypeT::typeId).release()));
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
      if (it != this->compsById.end() && nullptr != it->second)
        comp = it->second->Create();

      return comp;
    }

    /// \brief Create a new instance of a component, initialized with particular
    /// data.
    /// \param[in] _type Component id to create.
    /// \param[in] _data The data to populate the component instance with.
    /// \return Pointer to a component. Null if the component
    /// type could not be handled.
    public: std::unique_ptr<components::BaseComponent> New(
        const ComponentTypeId &_type, const components::BaseComponent *_data)
    {
      std::unique_ptr<components::BaseComponent> comp;

      if (nullptr == _data)
      {
        ignerr << "Requested to create a new component instance with null "
          << "data." << std::endl;
      }
      else if (_type != _data->TypeId())
      {
        ignerr << "The typeID of _type [" << _type << "] does not match the "
          << "typeID of _data [" << _data->TypeId() << "]." << std::endl;
      }
      else
      {
        auto it = this->compsById.find(_type);
        if (it != this->compsById.end() && nullptr != it->second)
          comp = it->second->Create(_data);
      }

      return comp;
    }

    /// \brief Create a new instance of a component storage.
    /// \param[in] _typeId Type of component which the storage will hold.
    /// \return Always returns nullptr.
    /// \deprecated Storages aren't necessary anymore.
    public: std::unique_ptr<ComponentStorageBase> IGN_DEPRECATED(6) NewStorage(
        const ComponentTypeId & /*_typeId*/)
    {
      return nullptr;
    }

    /// \brief Get all the registered component types by ID.
    /// return Vector of component IDs.
    public: std::vector<ComponentTypeId> TypeIds() const
    {
      std::vector<ComponentTypeId> types;

      // Return the list of all known component types.
      for (const auto &comp : this->compsById)
        types.push_back(comp.first);

      return types;
    }

    /// \brief Check if a component type has been registered.
    /// return True if registered.
    public: bool HasType(ComponentTypeId _typeId)
    {
      return this->compsById.find(_typeId) != this->compsById.end();
    }

    /// \brief Get a component's type name given its type ID.
    /// return Unique component name.
    public: std::string Name(ComponentTypeId _typeId) const
    {
      if (this->namesById.find(_typeId) != this->namesById.end())
        return namesById.at(_typeId);

      return "";
    }

    /// \brief A list of registered components where the key is its id.
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
    private: std::map<ComponentTypeId, ComponentDescriptorBase *> compsById;

    /// \brief A list of IDs and their equivalent names.
    public: std::map<ComponentTypeId, std::string> namesById;

    /// \brief Keep track of the runtime names for types and warn the user if
    /// they try to register different types with the same typeName.
    public: std::map<ComponentTypeId, std::string>
        runtimeNamesById;
  };

  /// \brief Static component registration macro.
  ///
  /// Use this macro to register components.
  ///
  /// \details Each time a plugin which uses a component is loaded, it tries to
  /// register the component again, so we prevent that.
  /// \param[in] _compType Component type name.
  /// \param[in] _classname Class name for component.
  #define IGN_GAZEBO_REGISTER_COMPONENT(_compType, _classname) \
  class GzSimComponents##_classname \
  { \
    public: GzSimComponents##_classname() \
    { \
      if (_classname::typeId != 0) \
        return; \
      using namespace gz;\
      using Desc = gazebo::components::ComponentDescriptor<_classname>; \
      gazebo::components::Factory::Instance()->Register<_classname>(\
        _compType, new Desc());\
    } \
  }; \
  static GzSimComponents##_classname\
    IgnitionGazeboComponentsInitializer##_classname;
}
}
}
}

#endif
