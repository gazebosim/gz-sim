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
#ifndef IGNITION_GAZEBO_COMPONENTFACTORY_HH_
#define IGNITION_GAZEBO_COMPONENTFACTORY_HH_

#include <any>
#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>
#include <ignition/common/Console.hh>
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
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

    /// \brief The ComponentManager constructs, deletes, and returns components.
    class IGNITION_GAZEBO_VISIBLE ComponentManager
    {
      /// \brief Create a component of a particular type.
      /// \param[in] _component Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      public: template<typename ComponentType>
              ComponentTypeId Register(const std::string &_typeName)
      {
        // Get a unique identifier to the component type
        const ComponentTypeId typeId = typeid(ComponentType).hash_code();

        // Create the component storage if one does not exist for
        // the component type.
        if (this->components.find(typeId) == this->components.end())
        {
          // Make sure that two types do not share the same name.
          if (this->componentNameMap.find(_typeName) ==
              this->componentNameMap.end())
          {
            igndbg << "Registering " << _typeName << " component type.\n";
            this->components[typeId].reset(
                new ComponentStorage<ComponentType>(_typeName));
            this->componentNameMap[_typeName] = typeId;
          }
          else
          {
            igndbg << "A component with the name [" << _typeName
                   << "] is already registered with a different type.\n";
            return kComponentTypeIdInvalid;
          }
        }

        return typeId;
      }

      /// \brief Create a component of a particular type.
      /// \param[in] _component Data used to construct the component.
      /// \return Key that uniquely identifies the component.
      public: template<typename ComponentType>
              ComponentKey CreateComponent(const ComponentType &_component)
              {
                // Get a unique identifier to the component type
                const ComponentTypeId typeId =
                  typeid(ComponentType).hash_code();
                std::cout << "Create type[" << typeId << "]\n";

                // Make sure the component has been registered.
                if (this->components.find(typeId) == this->components.end())
                {
                  igndbg << "Unable to create unregistered component.\n";
                  return {kComponentTypeIdInvalid, kComponentIdInvalid};
                }

                // Instantiate the new component.
                ComponentId id = this->components[typeId]->Create(_component);

                return std::make_pair(typeId, id);
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

      /// \brief Remove a component based on a key.
      /// \param[in] _key A key that uniquely identifies a component.
      /// \return The component associated with the key, or nullptr if the
      /// component could not be found.
      public: bool Remove(const ComponentKey &_key)
              {
                if (this->components.find(_key.first) != this->components.end())
                {
                  return this->components.at(_key.first)->Remove(_key.second);
                }
                return false;
              }

      /// \brief Get the ComponentTypeId associated with a string name.
      /// \param[in] _name Name of the component type.
      /// \return The ComponentTypeId if _name is Registered,
      /// kComponentTypeIdInvalid otherwise.
      public: ComponentTypeId Type(const std::string &_name)
      {
        std::map<std::string, ComponentTypeId>::const_iterator iter =
          this->componentNameMap.find(_name);

        if (iter != this->componentNameMap.end())
        {
          return iter->second;
        }
        return kComponentTypeIdInvalid;
      }

      /// \brief Map of component storage classes. The key is a component
      /// type id, and the value is a pointer to the component storage.
      private: std::map<ComponentTypeId,
               std::unique_ptr<ComponentStorageBase>> components;

      /// \brief Map of component names to component type ids.
      private: std::map<std::string, ComponentTypeId> componentNameMap;
    };
  }
}
#endif
