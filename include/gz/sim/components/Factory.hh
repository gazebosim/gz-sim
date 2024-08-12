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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/SingletonT.hh>
#include <gz/common/Util.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Types.hh>
#include <gz/utils/NeverDestroyed.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
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

  /// \brief A wrapper around uintptr_t to prevent implicit conversions.
  struct RegistrationObjectId
  {
    /// \brief Construct object from a pointer.
    /// \param[in] _ptr Arbitrary pointer.
    explicit RegistrationObjectId(void *_ptr)
        : id(reinterpret_cast<std::uintptr_t>(_ptr))
    {
    }

    /// \brief Construct object from a uintptr_t.
    /// \param[in] _ptr Arbitrary pointer address.
    explicit RegistrationObjectId(std::uintptr_t _ptrAddress)
        : id(_ptrAddress)
    {
    }

    /// \brief Equality comparison.
    /// \param[in] _other Other RegistrationObjectId object to compare against.
    bool operator==(const RegistrationObjectId &_other) const
    {
      return this->id == _other.id;
    }

    /// \brief Wrapped uintptr_t variable.
    std::uintptr_t id;
  };


  /// \brief A class to hold the queue of component descriptors registered by
  /// translation units. This queue is necessary to ensure that component
  /// creation continues to work after plugins are unloaded. The typical
  /// scenario this aims to solve is:
  ///  1. Plugin P1 registers component descripter for component C1.
  ///  2. Plugin P1 gets unloaded.
  ///  3. Plugin P2 registers a component descriptor for component C1 and tries
  ///     to create an instance of C1.
  /// When P1 gets unloaded, the destructor of the static component
  /// registration object calls Factory::Unregister which removes the component
  /// descriptor from the queue. Without this step, P2 would attempt to use
  /// the component descriptor created by P1 in step 3 and likely segfault
  /// because the memory associated with that descriptor has been deleted when
  /// P1 was unloaded.
  class ComponentDescriptorQueue
  {
    /// \brief Check if the queue is empty
    public: bool GZ_SIM_HIDDEN Empty()
    {
      return this->queue.empty();
    }

    /// \brief Add a component descriptor to the queue
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static component
    /// registration object created when calling GZ_SIM_REGISTER_COMPONENT.
    /// \param[in] _comp The component descriptor
    public: void GZ_SIM_HIDDEN Add(RegistrationObjectId _regObjId,
                     ComponentDescriptorBase *_comp)
    {
      this->queue.push_front({_regObjId, _comp});
    }

    /// \brief Remove a component descriptor from the queue. This also deletes
    /// memory allocated for the component descriptor by the static component
    /// registration object.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static component
    /// registration object created when calling GZ_SIM_REGISTER_COMPONENT.
    public: void GZ_SIM_HIDDEN Remove(RegistrationObjectId  _regObjId)
    {
      auto compIt = std::find_if(this->queue.rbegin(), this->queue.rend(),
                                 [&](const auto &_item)
                                 { return _item.first == _regObjId; });

      if (compIt != this->queue.rend())
      {
        ComponentDescriptorBase *compDesc = compIt->second;
        this->queue.erase(std::prev(compIt.base()));
        delete compDesc;
      }
    }

    /// \brief Create a component using the latest available component
    /// descriptor. This simply forward to ComponentDescriptorBase::Create
    /// \sa ComponentDescriptorBase::Create
    public: GZ_SIM_HIDDEN std::unique_ptr<BaseComponent> Create() const
    {
      if (!this->queue.empty())
      {
        return this->queue.front().second->Create();
      }
      return {};
    }

    /// \brief Create a component using the latest available component
    /// descriptor. This simply forward to ComponentDescriptorBase::Create
    /// \sa ComponentDescriptorBase::Create
    public: GZ_SIM_HIDDEN std::unique_ptr<BaseComponent> Create(
        const components::BaseComponent *_data) const
    {
      if (!this->queue.empty())
      {
        return this->queue.front().second->Create(_data);
      }
      return {};
    }

    /// \brief Queue of component descriptors registered by static registration
    /// objects.
    private: std::deque<std::pair<RegistrationObjectId,
                                  ComponentDescriptorBase *>> queue;
  };

  /// \brief A factory that generates a component based on a string type.
  class Factory
  {
    public: Factory(Factory &) = delete;
    public: Factory(const Factory &) = delete;
    public: void operator=(const Factory &) = delete;
    public: void operator=(Factory &&) = delete;

    /// \brief Get an instance of the singleton
    public: GZ_SIM_VISIBLE static Factory *Instance();

    /// \brief Register a component so that the factory can create instances
    /// of the component based on an ID.
    /// \param[in] _type Type of component to register.
    /// \param[in] _compDesc Object to manage the creation of ComponentTypeT
    ///  objects.
    /// \tparam ComponentTypeT Type of component to register.
    // Deprecated in favor of overload that takes _regObjId
    public: template <typename ComponentTypeT>
    void GZ_DEPRECATED(8) Register(const std::string &_type,
                                   ComponentDescriptorBase *_compDesc)
    {
      const char* typeDup = strdup(_type.c_str());
      this->Register<ComponentTypeT>(typeDup, _compDesc,
                                     RegistrationObjectId{nullptr});
    }
    /// \brief Register a component so that the factory can create instances
    /// of the component based on an ID.
    /// \param[in] _type Type of component to register.
    /// \param[in] _compDesc Object to manage the creation of ComponentTypeT
    ///  objects.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static component
    /// registration object created when calling GZ_SIM_REGISTER_COMPONENT.
    /// \tparam ComponentTypeT Type of component to register.
    // Deprecated in favor of overload that takes `const char *_type`
    public: template <typename ComponentTypeT>
    void GZ_DEPRECATED(8) Register(const std::string &_type,
        ComponentDescriptorBase *_compDesc, RegistrationObjectId _regObjId)
    {
      const char* typeDup = strdup(_type.c_str());
      this->Register<ComponentTypeT>(typeDup, _compDesc, _regObjId);
    }

    /// \brief Register a component so that the factory can create instances
    /// of the component based on an ID.
    /// \param[in] _type Type of component to register.
    /// \param[in] _compDesc Object to manage the creation of ComponentTypeT
    ///  objects.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static component
    /// registration object created when calling GZ_SIM_REGISTER_COMPONENT.
    /// \tparam ComponentTypeT Type of component to register.
    public: template <typename ComponentTypeT>
    void Register(const char *_type, ComponentDescriptorBase *_compDesc,
                  RegistrationObjectId  _regObjId)
    {
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
          return;
        }
      }

      // This happens at static initialization time, so we can't use common
      // console
      std::string debugEnv;
      gz::common::env("GZ_DEBUG_COMPONENT_FACTORY", debugEnv);

      if (debugEnv != "true")
      {
        gz::common::env("IGN_DEBUG_COMPONENT_FACTORY", debugEnv);
        if (debugEnv == "true")
        {
          std::cerr << "Environment variable [IGN_DEBUG_COMPONENT_FACTORY] "
                    << "is deprecated! Please use [GZ_DEBUG_COMPONENT_FACTORY]"
                    << "instead." << std::endl;
        }
      }

      if (debugEnv == "true")
      {
        std::cout << "Registering [" << ComponentTypeT::typeName << "]"
                  << std::endl;
      }

      // Keep track of all types
      this->compsById[ComponentTypeT::typeId].Add(_regObjId, _compDesc);
      namesById[ComponentTypeT::typeId] = ComponentTypeT::typeName;
      runtimeNamesById[ComponentTypeT::typeId] = runtimeName;
    }

    /// \brief Unregister a component so that the factory can't create instances
    /// of the component anymore.
    /// \tparam ComponentTypeT Type of component to unregister.
    // Deprecated in favor of overload that takes _regObjId
    public: template <typename ComponentTypeT>
    void GZ_DEPRECATED(8) Unregister()
    {
      this->Unregister<ComponentTypeT>(RegistrationObjectId{nullptr});
    }

    /// \brief Unregister a component so that the factory can't create instances
    /// of the component anymore.
    /// \tparam ComponentTypeT Type of component to unregister.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static component
    /// registration object created when calling GZ_SIM_REGISTER_COMPONENT.
    public: template<typename ComponentTypeT>
    void Unregister(RegistrationObjectId  _regObjId)
    {
      this->Unregister(ComponentTypeT::typeId, _regObjId);
    }

    /// \brief Unregister a component so that the factory can't create instances
    /// of the component anymore.
    /// \details This function will not reset the `typeId` static variable
    /// within the component type itself. Prefer using the templated
    /// `Unregister` function when possible.
    /// \param[in] _typeId Type of component to unregister.
    // Deprecated in favor of overload that takes _regObjId
    public: void GZ_DEPRECATED(8) Unregister(ComponentTypeId _typeId)
    {
      this->Unregister(_typeId, RegistrationObjectId{nullptr});
    }

    /// \brief Unregister a component so that the factory can't create instances
    /// of the component anymore.
    /// \details This function will not reset the `typeId` static variable
    /// within the component type itself. Prefer using the templated
    /// `Unregister` function when possible.
    /// \param[in] _typeId Type of component to unregister.
    /// \param[in] _regObjId An ID that identifies the registration object. This
    /// is generally derived from the `this` pointer of the static component
    /// registration object created when calling GZ_SIM_REGISTER_COMPONENT.
    public: void Unregister(ComponentTypeId _typeId,
                            RegistrationObjectId _regObjId)
    {
      auto it = this->compsById.find(_typeId);
      if (it != this->compsById.end())
      {
        it->second.Remove(_regObjId);

        if (it->second.Empty())
        {
          this->compsById.erase(it);
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
      if (it != this->compsById.end())
      {
        comp = it->second.Create();
      }
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
        gzerr << "Requested to create a new component instance with null "
          << "data." << std::endl;
      }
      else if (_type != _data->TypeId())
      {
        gzerr << "The typeID of _type [" << _type << "] does not match the "
          << "typeID of _data [" << _data->TypeId() << "]." << std::endl;
      }
      else
      {
        auto it = this->compsById.find(_type);
        if (it != this->compsById.end())
        {
          comp = it->second.Create(_data);
        }
      }

      return comp;
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

    // Since the copy constructors are deleted, we need to explicitly declare a
    // default constructor.
    private: Factory() = default;
    friend gz::utils::NeverDestroyed<Factory>;
    /// \brief A list of registered components where the key is its id.
    private: std::map<ComponentTypeId, ComponentDescriptorQueue> compsById;

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
  #define GZ_SIM_REGISTER_COMPONENT(_compType, _classname) \
  class GzSimComponents##_classname \
  { \
    public: GzSimComponents##_classname() \
    { \
      using namespace gz;\
      using Desc = sim::components::ComponentDescriptor<_classname>; \
      sim::components::Factory::Instance()->Register<_classname>(\
        _compType, new Desc(), sim::components::RegistrationObjectId(this));\
    } \
    public: GzSimComponents##_classname( \
                const GzSimComponents##_classname&) = delete; \
    public: GzSimComponents##_classname( \
                GzSimComponents##_classname&) = delete; \
    public: ~GzSimComponents##_classname() \
    { \
      using namespace gz; \
      sim::components::Factory::Instance()->Unregister<_classname>( \
          sim::components::RegistrationObjectId(this)); \
    } \
  }; \
  static GzSimComponents##_classname\
    GzSimComponentsInitializer##_classname;
}
}
}
}

#endif
