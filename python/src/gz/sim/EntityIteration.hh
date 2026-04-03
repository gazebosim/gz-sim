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
 */

#ifndef GZ_SIM_PYTHON_ENTITYITERATION_HH_
#define GZ_SIM_PYTHON_ENTITYITERATION_HH_

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <gz/sim/Types.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/detail/View.hh>
#include <gz/sim/detail/ComponentPybindRegistry.hh>

namespace py = pybind11;

namespace gz
{
namespace sim
{
namespace python
{

class PyCustomComponent : public gz::sim::components::BaseComponent
{
  public: PyCustomComponent(gz::sim::ComponentTypeId _typeId, py::object _obj)
      : typeId(_typeId), obj(std::move(_obj))
  {
  }

  public: ~PyCustomComponent() override = default;

  public: gz::sim::ComponentTypeId TypeId() const override
  {
    return this->typeId;
  }

  public: std::unique_ptr<gz::sim::components::BaseComponent> Clone() const override
  {
    py::module_ copy_module = py::module_::import("copy");
    py::object deepcopy = copy_module.attr("deepcopy");
    py::object cloned_obj = deepcopy(this->obj);
    
    return std::make_unique<PyCustomComponent>(this->typeId, cloned_obj);
  }

  public: py::object Object() const
  {
    return this->obj;
  }

  public: void SetObject(py::object _obj)
  {
    this->obj = std::move(_obj);
  }

  private: gz::sim::ComponentTypeId typeId;
  private: py::object obj;
};

struct ComponentCursor {
    gz::sim::Entity entity;
    std::vector<py::object> components;
};

class ComponentIterator {
    public: ComponentIterator(gz::sim::detail::View* _view,
                             const gz::sim::EntityComponentManager& _ecm,
                             const std::vector<gz::sim::ComponentTypeId>& _types)
        : view(_view), ecm(_ecm), types(_types)
    {
        if (this->view)
        {
            this->current = this->view->Entities().begin();
            this->end = this->view->Entities().end();
            this->cursor.components.resize(this->types.size());
        }
    }

    public: ComponentCursor* Next()
    {
        if (!this->view || this->current == this->end)
        {
            throw py::stop_iteration();
        }

        this->cursor.entity = *this->current;
        
        const auto &compData = this->view->EntityComponentData(this->cursor.entity);
        const auto &viewTypes = this->view->ComponentTypes();
        
        auto registry = gz::sim::python::ComponentPybindRegistry::Instance();
        for (size_t i = 0; i < this->types.size(); ++i)
        {
            auto typeId = this->types[i];
            
            size_t viewIdx = 0;
            for (auto t : viewTypes)
            {
                if (t == typeId) break;
                viewIdx++;
            }
            
            if (viewIdx < compData.size())
            {
                auto compBase = compData[viewIdx];
                auto customComp = dynamic_cast<const PyCustomComponent*>(compBase);
                if (customComp)
                {
                    this->cursor.components[i] = customComp->Object();
                }
                else
                {
                    auto getter = registry->Getter(typeId);
                    if (getter)
                    {
                        this->cursor.components[i] = getter(this->ecm, this->cursor.entity);
                    }
                    else
                    {
                        this->cursor.components[i] = py::none();
                    }
                }
            }
            else
            {
                this->cursor.components[i] = py::none();
            }
        }
        
        this->current++;
        return &this->cursor;
    }

    private: gz::sim::detail::View* view;
    private: const gz::sim::EntityComponentManager& ecm;
    private: std::vector<gz::sim::ComponentTypeId> types;
    private: std::set<gz::sim::Entity>::const_iterator current;
    private: std::set<gz::sim::Entity>::const_iterator end;
    private: ComponentCursor cursor;
};

class ECMPythonAccessor
{
  public: static const gz::sim::components::BaseComponent *Component(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId)
  {
    return _ecm.Component(_entity, _typeId);
  }

  public: static gz::sim::components::BaseComponent *Component(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId)
  {
    return _ecm.Component(_entity, _typeId);
  }

  public: static gz::sim::components::BaseComponent *CreateComponent(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity &_entity,
      gz::sim::ComponentTypeId _typeId,
      const gz::sim::components::BaseComponent *_data)
  {
    return _ecm.CreateComponent(_entity, _typeId, _data);
  }

  public: static ComponentIterator Each(
      gz::sim::EntityComponentManager &_ecm,
      const std::vector<gz::sim::ComponentTypeId> &_types)
  {
    auto baseViewMutexPair = _ecm.FindView(_types);
    auto baseViewPtr = baseViewMutexPair.first;
    gz::sim::detail::View* view = nullptr;

    if (nullptr != baseViewPtr)
    {
      view = static_cast<gz::sim::detail::View*>(baseViewPtr);
    }
    else
    {
      std::set<gz::sim::ComponentTypeId> compIds(_types.begin(), _types.end());
      auto newView = std::make_unique<gz::sim::detail::View>(compIds);
      
      for (const auto &vertex : _ecm.Entities().Vertices())
      {
        gz::sim::Entity entity = vertex.first;
        if (!_ecm.EntityMatches(entity, newView->ComponentTypes()))
          continue;

        std::vector<gz::sim::components::BaseComponent *> compPtrs;
        for (auto typeId : compIds)
        {
          compPtrs.push_back(const_cast<gz::sim::EntityComponentManager&>(_ecm).Component(entity, typeId));
        }
        
        newView->validData[entity] = compPtrs;
        newView->entities.insert(entity);
      }
      
      view = static_cast<gz::sim::detail::View*>(_ecm.AddView(_types, std::move(newView)));
    }

    return ComponentIterator(view, _ecm, _types);
  }
};

}  // namespace python
}  // namespace sim
}  // namespace gz

#endif
