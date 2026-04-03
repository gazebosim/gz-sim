/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <gz/sim/detail/ComponentPybindRegistry.hh>

namespace gz
{
namespace sim
{
namespace python
{

/////////////////////////////////////////////////
ComponentPybindRegistry *ComponentPybindRegistry::Instance()
{
  static gz::utils::NeverDestroyed<ComponentPybindRegistry> instance;
  return &instance.Access();
}

/////////////////////////////////////////////////
void ComponentPybindRegistry::Register(ComponentTypeId _typeId, uintptr_t _id,
                                       GetterFn _getter, SetterFn _setter)
{
  this->gettersAndSetters[_typeId].push_front({_id, _getter, _setter});
}

/////////////////////////////////////////////////
void ComponentPybindRegistry::Unregister(ComponentTypeId _typeId, uintptr_t _id)
{
  auto it = this->gettersAndSetters.find(_typeId);
  if (it != this->gettersAndSetters.end())
  {
    auto &queue = it->second;
    queue.erase(
        std::remove_if(queue.begin(), queue.end(),
                       [_id](const auto &_desc) { return _desc.id == _id; }),
        queue.end());

    if (queue.empty())
    {
      this->gettersAndSetters.erase(it);
    }
  }
}

/////////////////////////////////////////////////
ComponentPybindRegistry::GetterFn ComponentPybindRegistry::Getter(
    ComponentTypeId _typeId) const
{
  gztrace << "Getting Getter for " << _typeId << " size: " << this->gettersAndSetters.size() << std::endl;
  auto it = this->gettersAndSetters.find(_typeId);
  if (it == this->gettersAndSetters.end() || it->second.empty())
    return nullptr;
  return it->second.front().getter;
}

/////////////////////////////////////////////////
ComponentPybindRegistry::SetterFn ComponentPybindRegistry::Setter(
    ComponentTypeId _typeId) const
{
  auto it = this->gettersAndSetters.find(_typeId);
  if (it == this->gettersAndSetters.end() || it->second.empty())
    return nullptr;
  return it->second.front().setter;
}

}  // namespace python
}  // namespace sim
}  // namespace gz
