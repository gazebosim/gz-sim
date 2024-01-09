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
  ConfigLoader::loadMaterialFiles(&this->configLoader);
}

/////////////////////////////////////////////////
std::optional<MaterialParser::MaterialValues> MaterialParser::GetMaterialValues(
  const std::string& material)
{
  std::optional<MaterialValues> values = std::nullopt;
  std::map<std::string, ConfigNode *> scripts =
    this->configLoader.getAllConfigScripts();

  std::map<std::string, ConfigNode *> ::iterator it;

  for (it = scripts.begin(); it != scripts.end(); ++it) {
    std::string name = it->first;
    if (name.find(material) != std::string::npos) {
      if (!values) {
        values = MaterialValues();
      }
      ConfigNode * node = it->second;

      ConfigNode * techniqueNode = node->findChild("technique");
      if (techniqueNode) {
        ConfigNode * passNode = techniqueNode->findChild("pass");
        if (passNode) {
          ConfigNode * ambientNode = passNode->findChild("ambient");
          if (ambientNode) {
            std::vector<float> ambientValues;
            ambientNode->getColorValues(ambientValues, 3);
            values->ambient.emplace(gz::math::Color(ambientValues[0],
              ambientValues[1], ambientValues[2]));
          }

          ConfigNode * diffuseNode = passNode->findChild("diffuse");
          if (diffuseNode) {
            std::vector<float> diffuseValues;
            diffuseNode->getColorValues(diffuseValues, 3);
            values->diffuse.emplace(gz::math::Color(diffuseValues[0],
              diffuseValues[1], diffuseValues[2]));
          }

          ConfigNode * specularNode = passNode->findChild("specular");
          if (specularNode) {
            std::vector<float> specularValues;
            specularNode->getColorValues(specularValues, 4);
            // Using first four values for specular as
            // Gazebo doesn't support shininess
            values->specular.emplace(gz::math::Color(specularValues[0],
              specularValues[1], specularValues[2], specularValues[3]));
          }
        }
      }
      // \todo Handle dependent materials
    }
  }
  return values;
}
