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

#ifdef HAVE_PYBIND11

#include <gz/sim/python/ComponentPybindRegistry.hh>
#include <gz/sim/components/Factory.hh>

#include <algorithm>
#include <deque>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>

namespace gz
{
namespace sim
{
namespace python
{

class ComponentPybindRegistry::Implementation
{
  public: struct PybindDescriptor
  {
    uintptr_t id;
    std::string name;
    ComponentPybindRegistry::GetterFn getter;
    ComponentPybindRegistry::SetterFn setter;
  };

  public: mutable std::shared_mutex mutex;
  public: std::unordered_map<ComponentTypeId, std::deque<PybindDescriptor>>
      gettersAndSetters;
};

/////////////////////////////////////////////////
ComponentPybindRegistry::ComponentPybindRegistry()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

/////////////////////////////////////////////////
ComponentPybindRegistry::~ComponentPybindRegistry() = default;

/////////////////////////////////////////////////
ComponentPybindRegistry *ComponentPybindRegistry::Instance()
{
  static gz::utils::NeverDestroyed<ComponentPybindRegistry> instance;
  return &instance.Access();
}

/////////////////////////////////////////////////
std::string ComponentPybindRegistry::CleanName(const std::string &_name)
{
  const std::string prefix = "gz_sim_components.";
  if (_name.rfind(prefix, 0) == 0)
  {
    return _name.substr(prefix.length());
  }
  return _name;
}

/////////////////////////////////////////////////
void ComponentPybindRegistry::Register(ComponentTypeId _typeId, uintptr_t _id,
                                       GetterFn _getter, SetterFn _setter)
{
  std::string name = CleanName(
      gz::sim::components::Factory::Instance()->Name(_typeId));

  std::unique_lock<std::shared_mutex> lock(this->dataPtr->mutex);
  this->dataPtr->gettersAndSetters[_typeId].push_front(
      {_id, std::move(name), std::move(_getter), std::move(_setter)});
}

/////////////////////////////////////////////////
void ComponentPybindRegistry::Unregister(ComponentTypeId _typeId, uintptr_t _id)
{
  std::unique_lock<std::shared_mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->gettersAndSetters.find(_typeId);
  if (it != this->dataPtr->gettersAndSetters.end())
  {
    auto &queue = it->second;
    queue.erase(
        std::remove_if(queue.begin(), queue.end(),
                       [_id](const auto &_desc) { return _desc.id == _id; }),
        queue.end());

    if (queue.empty())
    {
      this->dataPtr->gettersAndSetters.erase(it);
    }
  }
}

/////////////////////////////////////////////////
ComponentPybindRegistry::GetterFn ComponentPybindRegistry::Getter(
    ComponentTypeId _typeId) const
{
  std::shared_lock<std::shared_mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->gettersAndSetters.find(_typeId);
  if (it == this->dataPtr->gettersAndSetters.end() || it->second.empty())
    return nullptr;
  return it->second.front().getter;
}

/////////////////////////////////////////////////
ComponentPybindRegistry::SetterFn ComponentPybindRegistry::Setter(
    ComponentTypeId _typeId) const
{
  std::shared_lock<std::shared_mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->gettersAndSetters.find(_typeId);
  if (it == this->dataPtr->gettersAndSetters.end() || it->second.empty())
    return nullptr;
  return it->second.front().setter;
}

/////////////////////////////////////////////////
std::optional<ComponentProxy> ComponentPybindRegistry::Proxy(
    ComponentTypeId _typeId) const
{
  std::shared_lock<std::shared_mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->gettersAndSetters.find(_typeId);
  if (it == this->dataPtr->gettersAndSetters.end() || it->second.empty())
    return std::nullopt;
  return ComponentProxy{it->second.front().name, _typeId};
}

/////////////////////////////////////////////////
bool ComponentPybindRegistry::HasBindings(ComponentTypeId _typeId) const
{
  std::shared_lock<std::shared_mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->gettersAndSetters.find(_typeId);
  return it != this->dataPtr->gettersAndSetters.end() && !it->second.empty();
}

}  // namespace python
}  // namespace sim
}  // namespace gz

#endif  // HAVE_PYBIND11
