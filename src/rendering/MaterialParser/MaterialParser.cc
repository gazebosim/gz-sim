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

#include <iostream>
#include <sstream>

#include "ConfigLoader.hh"
#include "MaterialParser.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
MaterialParser::MaterialParser()
{
  this->configLoader = new ConfigLoader(".material");
}

/////////////////////////////////////////////////
MaterialParser::~MaterialParser()
{
  delete this->configLoader;
}

/////////////////////////////////////////////////
void MaterialParser::Load(const std::string &_path)
{
  ConfigLoader::loadAllFiles(this->configLoader, _path);
}

/////////////////////////////////////////////////
std::vector<std::vector<float>> MaterialParser::GetMaterialValues(std::string material) const
{
  std::vector<std::vector<float>> values;
  std::map<std::string, ConfigNode *> scripts =
      this->configLoader->getAllConfigScripts();

  std::map<std::string, ConfigNode *>::iterator it;

  for (it = scripts.begin(); it != scripts.end(); ++it)
  {
    std::string name = it->first;

    if (name.find(material) != std::string::npos)
    {
      ConfigNode *node = it->second;

      ConfigNode *techniqueNode = node->findChild("technique");
      if (techniqueNode)
      {
        ConfigNode *passNode = techniqueNode->findChild("pass");
        if (passNode)
        {
          std::size_t index = name.rfind(" ");
          if (index != std::string::npos)
          {
            name = name.substr(index+1);
          }

          ConfigNode *ambientNode = passNode->findChild("ambient");
          if (ambientNode)
          {
            std::vector<float> ambientValues;
            ambientNode->getValuesInFloat(ambientValues);
            values.push_back(ambientValues);
          }

          ConfigNode *diffuseNode = passNode->findChild("diffuse");
          if (diffuseNode)
          {
            std::vector<float> diffuseValues;
            diffuseNode->getValuesInFloat(diffuseValues);
            values.push_back(diffuseValues);
          }

          ConfigNode *specularNode = passNode->findChild("specular");
          if (specularNode)
          {
            std::vector<float> specularValues;
            specularNode->getValuesInFloat(specularValues);
            values.push_back(specularValues);
          }
        }
      }
    }
  }
  return values;
}