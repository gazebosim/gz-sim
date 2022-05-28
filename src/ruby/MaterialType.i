/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

%module materialtype
%{
#include <gz/math/config.hh>
#include <gz/math/Export.hh>
#include <gz/math/MaterialType.hh>
%}

namespace gz
{
namespace math
{
  enum class MaterialType
    {
      STYROFOAM = 0,
      PINE,
      WOOD,
      OAK,
      PLASTIC,
      CONCRETE,
      ALUMINUM,
      STEEL_ALLOY,
      STEEL_STAINLESS,
      IRON,
      BRASS,
      COPPER,
      TUNGSTEN,
      UNKNOWN_MATERIAL
    };
}
}
