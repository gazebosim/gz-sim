/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "EntitySemantics.hh"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <map>
#include <optional>
#include <set>
#include <string>

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/SemanticCategory.hh"
#include "gz/sim/components/SemanticDescription.hh"
#include "gz/sim/components/SemanticTag.hh"

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{

// clang-format off
std::array kCategoryStrToInt = {
  std::pair{"object", 0u},
  std::pair{"robot", 1u},
  std::pair{"human", 2u},
  std::pair{"dynamic_object", 4u},
  std::pair{"static_object", 5u},
};
// clang-format on

std::optional<uint8_t> ConvertCategoryStrToInt(const std::string &_category)
{
  auto it =
      std::find_if(kCategoryStrToInt.begin(), kCategoryStrToInt.end(),
                   [&](const auto &_item) { return _category == _item.first; });

  if (it != kCategoryStrToInt.end())
  {
    return it->second;
  }
  return std::nullopt;
}
}  // namespace

//////////////////////////////////////////////////
void EntitySemantics::PreUpdate(const UpdateInfo &,
                                EntityComponentManager &_ecm)
{
  GZ_PROFILE("EntitySemantics::PreUpdate");

  // Store entities temporarily since it's not safe to add components in an
  // `EachNew` loop.
  std::map<Entity, uint8_t> entitiesToCategorize;
  std::map<Entity, std::string> entitiesToDescribe;
  std::map<Entity, std::set<std::string>> entitiesToTag;
  _ecm.EachNew<components::Name, components::ModelSdf>(
      [&](const Entity &_entity, const components::Name *_name,
          const components::ModelSdf *_modelSdf)
      {
        auto elem = _modelSdf->Data().Element();
        auto semanticsElem = elem->FindElement("gz:semantics");
        if (semanticsElem)
        {
          auto categoryElem = semanticsElem->FindElement("category");
          if (categoryElem)
          {
            auto categoryStr = categoryElem->Get<std::string>();
            auto category =
                ConvertCategoryStrToInt(common::lowercase(categoryStr));
            if (category)
            {
              entitiesToCategorize.emplace(_entity, *category);
            }
            else
            {
              gzerr << "Invalid category specified [" << categoryStr
                    << "] in model [" << _name->Data() << "]\n";
            }

            if (categoryElem->GetNextElement("category"))
            {
              // TODO(azeey) Include file and line number in error message.
              gzerr << "There should be only one <category> element for an "
                       "entity in model ["
                    << _name->Data() << "]\n";
            }
          }

          auto descriptionElem = semanticsElem->FindElement("description");
          if (descriptionElem)
          {
            auto description = descriptionElem->Get<std::string>();
            if (!description.empty())
            {
              entitiesToDescribe.emplace(_entity, description);
            }
            else
            {
              gzerr << "<description> cannot be an empty string"
                    << "] in model [" << _name->Data() << "]\n";
            }

            if (descriptionElem->GetNextElement("description"))
            {
              // TODO(azeey) Include file and line number in error message.
              gzerr << "There should be only one <description> element for an "
                       "entity in model ["
                    << _name->Data() << "]\n";
            }
          }

          for (auto tagElem = semanticsElem->FindElement("tag"); tagElem;
               tagElem = tagElem->GetNextElement("tag"))
          {
            auto tagStr = tagElem->Get<std::string>();
            if (tagStr.empty())
            {
              gzerr << "<tag> cannot be an empty string in model ["
                    << _name->Data() << "]\n";
            }
            else
            {
              entitiesToTag[_entity].insert(tagStr);
            }
          }
        }
        return true;
      });

  for (const auto &[entity, category] : entitiesToCategorize)
  {
    _ecm.SetComponentData<components::SemanticCategory>(entity, category);
  }

  for (const auto &[entity, description] : entitiesToDescribe)
  {
    _ecm.SetComponentData<components::SemanticDescription>(entity, description);
  }

  for (const auto &[entity, tags] : entitiesToTag)
  {
    _ecm.SetComponentData<components::SemanticTag>(entity, tags);
  }
}

GZ_ADD_PLUGIN(EntitySemantics, System, EntitySemantics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(EntitySemantics, "gz::sim::systems::EntitySemantics")
