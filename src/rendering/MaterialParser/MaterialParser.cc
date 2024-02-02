/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "MaterialParser.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
MaterialParser::MaterialParser()
  : configLoader()
{
}

/////////////////////////////////////////////////
void MaterialParser::Load()
{
  ConfigLoader::LoadMaterialFiles(&this->configLoader);
}

/////////////////////////////////////////////////
std::optional<MaterialParser::MaterialValues> MaterialParser::GetMaterialValues(
  const std::string& _material)
{
  std::optional<MaterialValues> values = std::nullopt;
  std::map<std::string, ConfigNode *> scripts =
    this->configLoader.GetAllConfigScripts();

  std::map<std::string, ConfigNode *> ::iterator it;

  for (it = scripts.begin(); it != scripts.end(); ++it)
  {
    std::string name = it->first;
    if (name.find(_material) != std::string::npos)
    {
      if (!values)
      {
        values = MaterialValues();
      }
      ConfigNode * node = it->second;

      ConfigNode * techniqueNode = node->FindChild("technique");
      if (techniqueNode)
      {
        ConfigNode * passNode = techniqueNode->FindChild("pass");
        if (passNode)
        {
          ConfigNode * ambientNode = passNode->FindChild("ambient");
          if (ambientNode)
          {
            gz::math::Color ambientValues;
            ambientNode->GetColorValues(ambientValues, 3);
            values->ambient.emplace(ambientValues);
          }

          ConfigNode * diffuseNode = passNode->FindChild("diffuse");
          if (diffuseNode)
          {
            gz::math::Color diffuseValues;
            diffuseNode->GetColorValues(diffuseValues, 3);
            values->diffuse.emplace(diffuseValues);
          }

          ConfigNode * specularNode = passNode->FindChild("specular");
          if (specularNode)
          {
            gz::math::Color specularValues;
            specularNode->GetColorValues(specularValues, 4);
            // Using first four values for specular as
            // Gazebo doesn't support shininess
            values->specular.emplace(specularValues);
          }
        }
      }
      // \todo Handle dependent materials
    }
  }
  return values;
}
