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

#ifndef RENDERING__MATERIALPARSER__MATERIALPARSER_HH_
#define RENDERING__MATERIALPARSER__MATERIALPARSER_HH_

#include <string>
#include <filesystem>
#include <vector>


namespace gz
{
  namespace sim
  {
    class ConfigNode;
    class ConfigLoader;

    class MaterialParser
    {
public:
      MaterialParser();

public:
      ~MaterialParser();

public:
      void Load(const std::string & _path);

public:
      void Load() {Load(std::filesystem::current_path().string());}

public:
      std::vector < std::vector < float >> GetMaterialValues(std::string material) const;

private:
      ConfigLoader * configLoader = nullptr;
    };
  }  // namespace sim
}  // namespace gz

#endif  // RENDERING__MATERIALPARSER__MATERIALPARSER_HH_
