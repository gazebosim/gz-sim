/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef IGNITION_GAZEBO_COMPONENT_MANAGER_HH_
#define IGNITION_GAZEBO_COMPONENT_MANAGER_HH_

#include <any>
#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>
#include <ignition/common/Console.hh>
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityQuery.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Forward declarations.
    class EntityComponentManagerPrivate;

    /// \cond
    /// \brief All component instances of the same type are stored
    /// squentially in memory. This is a base class for storing components
    /// of a particular type.
    class IGNITION_GAZEBO_HIDDEN ComponentStorageBase
    {
      /// \brief Constructor
      /// \param[in] _name Name associated with the component storage type.
      public: ComponentStorageBase(const std::string &_name)
              : name(_name)
      {
      }

      /// \brief Create a new component using the provided data.
      /// \param[in] _data Data used to construct the component.
      /// \return Id of the new component. kComponentIdInvalid is returned
      /// if the component could not be created.
      public: virtual ComponentId Create(const std::any &_data) = 0;

      /// \brief Remove a component based on an id.
      /// \param[in] _id Id of the component to remove.
      /// \return True if the component was removed.
      public: virtual bool Remove(const ComponentId _id) = 0;

      /// \brief Remove all components
      public: virtual void RemoveAll() = 0;

      /// \brief Get a component based on an id.
      /// \param[in] _id Id of the component to get.
      /// \return A pointer to the component, or nullptr if the component
      /// could not be found.
      public: virtual const void *Component(const ComponentId _id) const = 0;

      /// \brief Get the name associated with the component storage type.
      /// \return Name associated with the component storage type.
      public: const std::string &Name() const
      {
        return this->name;
      }

      /// \brief Mutex used to prevent data corruption.
      protected: mutable std::mutex mutex;

      /// \brief Name of the component.
      private: std::string name;
    };

    /// \brief Templated implementation of component storage.
    template<typename ComponentType>
    class IGNITION_GAZEBO_HIDDEN ComponentStorage : public ComponentStorageBase
    {
      /// \brief Constructor
      /// \param[in] _name Name associated with the component storage type.
      public: explicit ComponentStorage(const std::string &_name)
              : ComponentStorageBase(_name)
      {
      }

      // Documentation inherited.
      public: bool Remove(const ComponentId _id) override final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        std::map<ComponentId, int>::iterator iter = this->idMap.find(_id);
        if (iter != this->idMap.end())
        {
          if (this->components.size() > 1)
          {
            std::swap(this->components[iter->second],
                      this->components.back());

            // Fix the id mapping.
            for (std::map<ComponentId, int>::iterator idIter =
                 this->idMap.begin(); idIter != this->idMap.end(); ++idIter)
            {
              if (static_cast<unsigned int>(idIter->second) ==
                  this->components.size()-1)
              {
                idIter->second = iter->second;
              }
            }
          }
          // Remove the component.
          this->components.pop_back();

          // Remove the id mapping.
          this->idMap.erase(iter);
          return true;
        }
        return false;
      }

      // Documentation inherited.
      public: void RemoveAll() override final
      {
        this->idCounter = 0;
        this->idMap.clear();
        this->components.clear();
      }

      // Documentation inherited.
      public: ComponentId Create(const std::any &_data) override final
      {
        ComponentId result = kComponentIdInvalid;

        try
        {
          std::lock_guard<std::mutex> lock(this->mutex);
          const ComponentType &data = std::any_cast<ComponentType>(_data);
          result = idCounter++;
          this->idMap[result] = this->components.size();
          this->components.push_back(std::move(ComponentType(data)));
        }
        catch(std::bad_any_cast &_cast)
        {
          ignerr << "Unable to create a component. "
                 << _cast.what() << std::endl;
        }

        return result;
      }

      // Documentation inherited.
      public: const void *Component(const ComponentId _id) const override final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        std::map<ComponentId, int>::const_iterator iter = this->idMap.find(_id);

        if (iter != this->idMap.end())
        {
          return static_cast<const void *>(&this->components.at(iter->second));
        }
        return nullptr;
      }

      /// \brief The id counter is used to get unique ids within this
      /// storage class.
      private: ComponentId idCounter = 0;

      /// \brief Map of ComponentId to Components (see the components vector).
      private: std::map<ComponentId, int> idMap;

      /// \brief Sequential storage of components.
      public: std::vector<ComponentType> components;
    };
    /// \endcond

    /// \brief The EntityComponentManager constructs, deletes, and returns
    /// components.
    class IGNITION_GAZEBO_VISIBLE EntityComponentManager
    {
      /// \brief Constructor
      public: EntityComponentManager();

      /// \brief Destructor
      public: ~EntityComponentManager();

      /// \brief Creates a new Entity.
      /// \return An id for the Entity, or kNullEntity on failure.
      public: EntityId CreateEntity();

      /// \brief Get the number of entities on the server.
      /// \return Entity count.
      public: size_t EntityCount() const;

      /// \brief Delete an existing Entity.
      /// \returns True if the Entity existed and was deleted.
      public: bool EraseEntity(const EntityId _id);

      /// \brief Delete all entities.
      public: void EraseEntities();

      /// \brief Get whether an Entity exists.
      /// \param[in] _id Entity id to confirm.
      /// \return True if the Entity exists.
      public: bool HasEntity(EntityId _id) const;

      /// \brief Check whether an entity has a specific component.
      /// \param[in] _id Id of the entity to check.
      /// \param[in] _key The component to check.
      /// \return True if the component key belongs to the entity.
      public: bool EntityHasComponent(const EntityId _id,
                  const ComponentKey &_key) const;

      /// \brief Check whether an entity has a specific component type.
      /// \param[in] _id Id of the entity to check.
      /// \param[in] _typeId Component type id to check.
      /// \return True if the entity exists and has at least one component
      /// with the provided type.
      public: bool EntityHasComponentType(const EntityId _id,
                  const ComponentTypeId &_typeId) const;

      /// \brief Add an entity query. An internal copy of the query is made,
      /// and an Id of the internal query is returned.
      /// \param[in] _query The query to add.
      /// \return Id of the newly added query.
      public: EntityQueryId AddQuery(const EntityQuery &_query);

      /// \brief Get an entity query.
      /// \param[in] _id Id of the EntityQuery to retrieved.
      /// \return Optional reference to the query. The return value will be
      /// std::nullopt if the entity query does not exist.
      public: const std::optional<std::reference_wrapper<EntityQuery>> Query(
                  const EntityQueryId _id) const;

      /// \brief Remove a component based on a key.
      /// \param[in] _id Id of the entity.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return True if the entity and component existed and was removed.
      public: bool RemoveComponent(
                  const EntityId _id, const ComponentKey &_key);

      /// \brief Get the ComponentTypeId associated with a string name.
      /// \param[in] _name Name of the component type.
      /// \return The ComponentTypeId if _name is Registered,
      /// kComponentTypeIdInvalid otherwise.
      public: ComponentTypeId Type(const std::string &_name) const;

      /// \brief Register a component type, and associate a name to the type.
      /// \param[in] _name Unique name for the type..
      /// \return An id that uniquely identifies the component type.
      public: template<typename ComponentType>
              ComponentTypeId RegisterComponentType(const std::string &_name)
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = typeid(ComponentType).hash_code();

        // Create the component storage if one does not exist for
        // the component type.
        if (this->components.find(typeId) == this->components.end())
        {
          // Make sure that two types do not share the same name.
          if (this->componentNameMap.find(_name) ==
              this->componentNameMap.end())
          {
            igndbg << "Registering " << _name << " component type.\n";
            this->components[typeId].reset(
                new ComponentStorage<ComponentType>(_name));
            this->componentNameMap[_name] = typeId;
          }
          else
          {
            igndbg << "A component with the name [" << _name
                   << "] is already registered with a different type.\n";
            return kComponentTypeIdInvalid;
          }
        }

        return typeId;
      }

      /// \brief Create a component of a particular type.
      /// \param[in] _entityId Id of the Entity that will be associated with
      /// the component.
      /// \param[in] _component Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      public: template<typename ComponentType>
              ComponentKey CreateComponent(const EntityId _entityId,
                  const ComponentType &_component)
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = typeid(ComponentType).hash_code();

        return this->CreateComponentImplementation(_entityId, typeId,
            _component);
      }

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: template<typename ComponentType>
              const ComponentType *Component(const EntityId _id) const
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = typeid(ComponentType).hash_code();

        return static_cast<const ComponentType*>(
            this->ComponentImplementation(_id, typeId));
      }

      /// \brief Get a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: template<typename ComponentType>
              const ComponentType *Component(const ComponentKey &_key) const
      {
        if (this->components.find(_key.first) != this->components.end())
        {
          return static_cast<const ComponentType *>(
              this->components.at(_key.first)->Component(_key.second));
        }
        return nullptr;
      }

      /// \brief Implmentation of CreateComponent.
      /// \param[in] _entityId Id of the Entity that will be associated with
      /// the component.
      /// \param[in] _componentTypeId Id of the component type.
      /// \param[in] _data Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      private: ComponentKey CreateComponentImplementation(
                   const EntityId _entityId,
                   const ComponentTypeId _componentTypeId,
                   const std::any &_data);

      private: const void *ComponentImplementation(const EntityId _id,
                   const ComponentTypeId _type) const;

      /// \brief Map of component storage classes. The key is a component
      /// type id, and the value is a pointer to the component storage.
      private: std::map<ComponentTypeId,
               std::unique_ptr<ComponentStorageBase>> components;

      /// \brief Map of component names to component type ids.
      private: std::map<std::string, ComponentTypeId> componentNameMap;

      /// \brief Private data pointer.
      private: std::unique_ptr<EntityComponentManagerPrivate> dataPtr;
    };
  }
}
#endif
