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

%module material
%{
#include <gz/math/config.hh>
#include <gz/math/Export.hh>
#include <gz/math/Material.hh>
#include <gz/math/MaterialType.hh>
%}

%include "std_string.i"
%include "std_map.i"

%template(map_material_type) std::map<int,
                                      gz::math::Material>;

namespace gz
{
namespace math
{
    class Material
    {
      %rename("%(undercase)s", %$isfunction, %$ismember, %$not %$isconstructor) "";
      public: Material();
      public: explicit Material(const MaterialType _type);
      public: explicit Material(const std::string &_typename);
      public: explicit Material(const double _density);
      public: Material(const Material &_material);
      public: ~Material();
      public: static const std::map<int, Material> &Predefined();
      public: void SetToNearestDensity(
                  const double _value,
                  const double _epsilon = std::numeric_limits<double>::max());
      public: bool operator==(const Material &_material) const;
      public: bool operator!=(const Material &_material) const;
      public: MaterialType Type() const;
      public: void SetType(const MaterialType _type);
      public: std::string Name() const;
      public: void SetName(const std::string &_name);
      public: double Density() const;
      public: void SetDensity(const double _density);
    };
}
}
