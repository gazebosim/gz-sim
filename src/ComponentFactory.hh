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
#include <typeinfo>
#include <utility>
#include <vector>
#include <ignition/common/Console.hh>
#include "ignition/gazebo/Export.hh"

namespace ignition
{
  namespace gazebo
  {
    using ComponentId = int;
    using ComponentTypeId = std::size_t;
    using ComponentKey = std::pair<ComponentTypeId, ComponentId>;

    /// \brief Id that indicates an invalid component.
    const ComponentId kComponentIdInvalid = -1;

    class IGNITION_GAZEBO_HIDDEN ComponentStorageBase
    {
      public: virtual ComponentId Create(const std::any &_data) = 0;
      public: virtual void Remove(const ComponentId _id) = 0;
      public: virtual const void *Component(const ComponentId _id) const = 0;
    };

    template<typename ComponentType>
    class IGNITION_GAZEBO_HIDDEN ComponentStorage : public ComponentStorageBase
    {
      public: void Remove(const ComponentId _id) override final
      {
        // lock!
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
        }
      }

      public: ComponentId Create(const std::any &_data) override final
      {
        ComponentId result = kComponentIdInvalid;

        // lock!
        /// \todo(nkoenig) Is this try necessary?
        try
        {
          const ComponentType &data = std::any_cast<ComponentType>(_data);
          result = idCounter++;
          this->idMap[result] = this->components.size();
          this->components.push_back(std::move(ComponentType(data)));
        }
        catch(std::bad_any_cast &_cast)
        {
          ignerr << "Unable to create a component. "
                 << _cast.what() << std::endl;
          return result;
        }

        return result;
      }

      public: const void *Component(const ComponentId _id) const override final
      {
        // lock!
        if (this->idMap.find(_id) != this->idMap.end())
        {
          return static_cast<const void *>(&this->components.at(_id));
        }
        return nullptr;
      }

      private: ComponentId idCounter = 0;
      private: std::map<ComponentId, int> idMap;
      public: std::vector<ComponentType> components;
    };

    class IGNITION_GAZEBO_HIDDEN ComponentFactory
    {
      public: template<typename ComponentType>
              ComponentKey CreateComponent(const ComponentType &_component)
              {
                // Get a unique identifier to the component type
                const ComponentTypeId typeId =
                  typeid(ComponentType).hash_code();

                // Create the component storage if one does not exist for
                // the component type.
                if (this->components.find(typeId) == this->components.end())
                {
                  this->components[typeId].reset(
                      new ComponentStorage<ComponentType>());
                }

                // Instantiate the new component.
                ComponentId id = this->components[typeId]->Create(_component);

                return std::make_pair(typeId, id);
              }

      public: template<typename ComponentType>
              const ComponentType *Component(const ComponentKey &_key) const
              {
                if (this->components.find(_key.first) != this->components.end())
                {
                  return static_cast<const ComponentType*>(
                      this->components.at(_key.first)->Component(_key.second));
                }
                return nullptr;
              }

      private: std::map<ComponentTypeId,
               std::unique_ptr<ComponentStorageBase>> components;
    };
  }
}
#endif
