/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#include "gz/sim/ecs/ComponentTypeRegistry.hh"

namespace gz::sim::ecs
{
  ComponentTypeRegistry &ComponentTypeRegistry::Instance()
  {
    static ComponentTypeRegistry inst;
    return inst;
  }

  const ComponentTypeInfo *ComponentTypeRegistry::Get(
      ComponentTypeId _id) const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto *e : this->entries_)
    {
      if (e->id == _id)
        return e;
    }
    return nullptr;
  }

  const ComponentTypeInfo *ComponentTypeRegistry::Install(
      ComponentTypeInfo &&_info)
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto *e : this->entries_)
    {
      if (e->id == _info.id)
        return e;
    }
    auto *heap = new ComponentTypeInfo(std::move(_info));
    this->entries_.push_back(heap);
    return heap;
  }

  size_t ComponentTypeRegistry::Size() const
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    return this->entries_.size();
  }

  void ComponentTypeRegistry::ClearForTesting()
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto *e : this->entries_)
      delete e;
    this->entries_.clear();
  }
}
