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

#include <optional>
#include <string>

#include <gz/math/Color.hh>

#include "ConfigLoader.hh"

namespace gz
{
namespace sim
{
class MaterialParser
{
  public:
  struct MaterialValues
    {
      std::optional<math::Color> ambient;
      std::optional<math::Color> diffuse;
      std::optional<math::Color> specular;
    };

  MaterialParser();

  void Load();

  std::optional<MaterialValues> GetMaterialValues(const std::string& _material);

  private:
  ConfigLoader configLoader;
};
}  // namespace sim
}  // namespace gz

#endif  // RENDERING__MATERIALPARSER__MATERIALPARSER_HH_
