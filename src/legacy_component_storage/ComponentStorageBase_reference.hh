/*
 * Reference reconstruction of ComponentStorageBase.hh (historically
 * include/ignition/gazebo/detail/ComponentStorageBase.hh), which no longer
 * exists in this tree -- the per-component-type ComponentStorage<T> template
 * (a reserve(100)-and-grow-by-100 std::vector, whose reallocation triggered a
 * full EntityComponentManager::RebuildViews()) was replaced by a per-entity
 * `std::unordered_map<Entity, std::vector<std::unique_ptr<BaseComponent>>>`
 * (src/EntityComponentManager.cc), where growth only ever moves owning
 * pointers, never the components themselves, and RebuildViews() is now only
 * called on a full entity-removal reset -- normal component creation uses
 * incremental `view->MarkEntityToAdd()` instead. That refactor independently
 * eliminated the reallocation-triggered full rebuild PR #927's area (and this
 * optimization) targets, via a different mechanism than a vector-to-deque
 * swap.
 *
 * The storage/reallocation logic below is copied verbatim (algorithm; the
 * BaseComponent virtual-dispatch scaffolding around it is dropped since this
 * reconstruction only needs to demonstrate the storage container's own
 * behavior, not the full polymorphic component hierarchy) from the real
 * historical source (gz-sim/ignition-gazebo commit 34ac465, the base this
 * instance's tooling profiled against).
 *
 * Not referenced by any build file. Not built.
 */

#pragma once

#include <map>
#include <mutex>
#include <utility>
#include <vector>

namespace legacy_component_storage
{

using ComponentId = int;

// Templated implementation of component storage, copied verbatim (algorithm)
// from the real ComponentStorage<ComponentTypeT>.
template<typename ComponentTypeT>
class ComponentStorageRef
{
  public: explicit ComponentStorageRef()
  {
    // Reserve a chunk of memory for the components. The size here will
    // effect how often Views are rebuilt when
    // EntityComponentManager::CreateComponent() is called.
    this->components.reserve(100);
  }

  public: bool Remove(const ComponentId _id)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    auto iter = this->idMap.find(_id);
    if (iter != this->idMap.end())
    {
      if (this->components.size() > 1)
      {
        std::swap(this->components[iter->second], this->components.back());
        for (auto idIter = this->idMap.begin(); idIter != this->idMap.end(); ++idIter)
        {
          if (static_cast<unsigned int>(idIter->second) == this->components.size() - 1)
          {
            idIter->second = iter->second;
          }
        }
      }
      this->components.pop_back();
      this->idMap.erase(iter);
      return true;
    }
    return false;
  }

  public: void RemoveAll()
  {
    this->idCounter = 0;
    this->idMap.clear();
    this->components.clear();
  }

  // Returns {id, expanded}: expanded is true iff this call's push_back
  // triggered a capacity-exceeding reallocation -- the signal
  // EntityComponentManager::CreateComponentImplementation used to decide
  // whether to run a full RebuildViews().
  public: std::pair<ComponentId, bool> Create(const ComponentTypeT *_data)
  {
    ComponentId result;
    bool expanded = false;
    if (this->components.size() == this->components.capacity())
    {
      this->components.reserve(this->components.capacity() + 100);
      expanded = true;
    }

    std::lock_guard<std::mutex> lock(this->mutex);
    result = this->idCounter++;
    this->idMap[result] = this->components.size();
    this->components.push_back(std::move(ComponentTypeT(*_data)));

    return {result, expanded};
  }

  public: const ComponentTypeT *Component(const ComponentId _id) const
  {
    return const_cast<ComponentStorageRef<ComponentTypeT> *>(this)->Component(_id);
  }

  public: ComponentTypeT *Component(const ComponentId _id)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    auto iter = this->idMap.find(_id);
    if (iter != this->idMap.end())
    {
      return &this->components.at(iter->second);
    }
    return nullptr;
  }

  public: ComponentTypeT *First()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->components.empty())
      return &this->components[0];
    return nullptr;
  }

  private: ComponentId idCounter = 0;
  private: std::map<ComponentId, int> idMap;
  public: std::vector<ComponentTypeT> components;
  private: mutable std::mutex mutex;
};

}  // namespace legacy_component_storage
