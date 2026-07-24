/*
 * Optimized reconstruction of ComponentStorageBase.hh: see
 * ComponentStorageBase_reference.hh for provenance, and the note that this
 * whole directory is a not-built reference reconstruction -- the current
 * EntityComponentManager already eliminated the bug this fixes via a
 * different (per-entity, unique_ptr-indirected) storage redesign.
 *
 * The historical ComponentStorage<T>::Create() reserved a std::vector to 100
 * and grew it by +100 whenever it filled, returning an "expanded" flag on
 * that reallocation. EntityComponentManager::CreateComponentImplementation
 * reacted to "expanded" by running a full RebuildViews() (O(views x entities
 * x types)), so populating a world of E entities triggered ~E/100 full
 * rebuilds, each O(E): O(E^2) world population.
 *
 * Backing the same storage with std::deque instead of std::vector removes
 * the trigger entirely: deque never relocates existing elements on growth
 * (it grows by allocating new fixed-size blocks and chaining them), so
 * "expanded" is always false, and CreateComponentImplementation only ever
 * runs the incremental per-entity view update. O(E^2) -> O(E), with the same
 * ids, same stored values, and the same Component(id) semantics -- deque
 * also gives a stable pointer per element like the vector case's `.at()` did
 * (until the vector *reallocated*, at which point vector pointers dangled;
 * deque pointers never do, which incidentally fixes that hazard too).
 */

#pragma once

#include <deque>
#include <map>
#include <mutex>
#include <utility>

namespace legacy_component_storage
{

using ComponentId = int;

template<typename ComponentTypeT>
class ComponentStorageOurs
{
  public: explicit ComponentStorageOurs()
  {
    // std::deque keeps stable element addresses on growth, so appending
    // never invalidates existing pointers -> no capacity-triggered full
    // RebuildViews(). No reserve() call needed; deque doesn't have one.
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

  public: std::pair<ComponentId, bool> Create(const ComponentTypeT *_data)
  {
    ComponentId result;
    const bool expanded = false;  // deque growth never reallocates existing elements

    std::lock_guard<std::mutex> lock(this->mutex);
    result = this->idCounter++;
    this->idMap[result] = this->components.size();
    this->components.push_back(std::move(ComponentTypeT(*_data)));

    return {result, expanded};
  }

  public: const ComponentTypeT *Component(const ComponentId _id) const
  {
    return const_cast<ComponentStorageOurs<ComponentTypeT> *>(this)->Component(_id);
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
  public: std::deque<ComponentTypeT> components;
  private: mutable std::mutex mutex;
};

}  // namespace legacy_component_storage
