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
#ifndef IGNITION_GAZEBO_DETAIL_COMPONENTSTORAGEBASE_HH_
#define IGNITION_GAZEBO_DETAIL_COMPONENTSTORAGEBASE_HH_

#include <unordered_map>
#include <utility>
#include <vector>
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    /// \brief All component instances of the same type are stored
    /// squentially in memory. This is a base class for storing components
    /// of a particular type.
    class IGNITION_GAZEBO_HIDDEN ComponentStorageBase
    {
      /// \brief Constructor
      public: ComponentStorageBase() = default;

      /// \brief Destructor
      public: virtual ~ComponentStorageBase() = default;

      /// \brief Create a new component using the provided data.
      /// \param[in] _data Data used to construct the component.
      /// \return Id of the new component, and whether the components array
      /// was expanded. kComponentIdInvalid is returned
      /// if the component could not be created.
      public: virtual std::pair<ComponentId, bool> Create(
                  const components::BaseComponent *_data) = 0;

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
      public: virtual const components::BaseComponent *Component(
                  const ComponentId _id) const = 0;

      /// \brief Get a mutable component based on an id.
      /// \param[in] _id Id of the component to get.
      /// \return A pointer to the component, or nullptr if the component
      /// could not be found.
      public: virtual components::BaseComponent *Component(
                  const ComponentId _id) = 0;

      /// \brief Get the first component.
      /// \return First component or nullptr if there are no components.
      public: virtual components::BaseComponent *First() = 0;

      /// \brief Mutex used to prevent data corruption.
      protected: mutable std::mutex mutex;
    };

    /// \brief Templated implementation of component storage.
    template<typename ComponentTypeT>
    class IGNITION_GAZEBO_HIDDEN ComponentStorage : public ComponentStorageBase
    {
      /// \brief Constructor
      public: explicit ComponentStorage()
              : ComponentStorageBase()
      {
        // Reserve a chunk of memory for the components. The size here will
        // effect how often Views are rebuilt when
        // EntityComponentManager::CreateComponent() is called.
        //
        // Views would be rebuilt if the components vector capacity is
        // exceeded after an EntityComponentManager::Each call has already
        // been executed.
        //
        // See also this class's Create() function, which expands the value
        // of components vector whenever the capacity is reached.
        this->components.reserve(100);
      }

      // Documentation inherited.
      public: bool Remove(const ComponentId _id) final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        // Get an iterator to the component that should be removed.
        auto iter = this->idMap.find(_id);

        // Make sure the component exists.
        if (iter != this->idMap.end())
        {
          // Handle the case where there are more components than the
          // component to be removed
          if (this->components.size() > 1)
          {
            // Swap the component to be removed with the component at the
            // back of the vector.
            std::swap(this->components[iter->second],
                      this->components.back());

            // After the swap, we have to fix all the id mappings.
            for (auto idIter =this->idMap.begin();
                idIter != this->idMap.end(); ++idIter)
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
      public: void RemoveAll() final
      {
        this->idCounter = 0;
        this->idMap.clear();
        this->components.clear();
      }

      // Documentation inherited.
      public: std::pair<ComponentId, bool> Create(
                  const components::BaseComponent *_data) final
      {
        ComponentId result;  // = kComponentIdInvalid;
        bool expanded = false;
        if (this->components.size() == this->components.capacity())
        {
          this->components.reserve(this->components.capacity() + 100);
          expanded = true;
        }

        std::lock_guard<std::mutex> lock(this->mutex);
        // cppcheck-suppress unmatchedSuppression
        // cppcheck-suppress postfixOperator
        result = this->idCounter++;
        this->idMap[result] = this->components.size();
        // Copy the component
        this->components.push_back(std::move(
              ComponentTypeT(*static_cast<const ComponentTypeT *>(_data))));

        return {result, expanded};
      }

      // Documentation inherited.
      public: const components::BaseComponent *Component(
                  const ComponentId _id) const final
      {
        return static_cast<const components::BaseComponent *>(
            const_cast<ComponentStorage<ComponentTypeT> *>(
              this)->Component(_id));
      }

      public: components::BaseComponent *Component(const ComponentId _id) final
      {
        std::lock_guard<std::mutex> lock(this->mutex);

        auto iter = this->idMap.find(_id);

        if (iter != this->idMap.end())
        {
          return static_cast<components::BaseComponent *>(
              &this->components.at(iter->second));
        }
        return nullptr;
      }

      // Documentation inherited.
      public: components::BaseComponent *First() final
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        if (!this->components.empty())
          return static_cast<components::BaseComponent *>(&this->components[0]);
        return nullptr;
      }

      /// \brief The id counter is used to get unique ids within this
      /// storage class.
      private: ComponentId idCounter = 0;

      /// \brief Map of ComponentId to Components (see the components vector).
      private: std::unordered_map<ComponentId, int> idMap;

      /// \brief Sequential storage of components.
      public: std::vector<ComponentTypeT> components;
    };
    }
  }
}

#endif
