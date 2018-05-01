/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "ignition/math/Material.hh"
#include "ignition/math/Helpers.hh"

// Placing the kMaterialData in a separate file for conveniece and clarity.
#include "MaterialType.hh"

using namespace ignition;
using namespace math;

// Initialize the static map of Material objects based on the kMaterialData.
static const std::map<MaterialType, Material> kMaterials = []()
{
  std::map<MaterialType, Material> matMap;

  for (const std::pair<MaterialType, MaterialData> &mat : kMaterialData)
  {
    matMap[mat.first].SetType(mat.first);
    matMap[mat.first].SetName(mat.second.name);
    matMap[mat.first].SetDensity(mat.second.density);
  }

  return matMap;
}();

// Private data for the Material class
class ignition::math::MaterialPrivate
{
  /// \brief The material type.
  public: MaterialType type = MaterialType::UNKNOWN_MATERIAL;

  /// \brief Name of the material. This will match the names
  /// used in MaterialType, but in lowercase.
  public: std::string name = "";

  /// \brief Density value of the material in kg/m^3.
  public: double density = -1;
};

///////////////////////////////
Material::Material()
: dataPtr(new MaterialPrivate)
{
}

///////////////////////////////
Material::Material(const MaterialType _type)
: dataPtr(new MaterialPrivate)
{
  if (kMaterials.find(_type) != kMaterials.end())
  {
    this->dataPtr->type = _type;
    this->dataPtr->name = kMaterials.at(_type).Name();
    this->dataPtr->density = kMaterials.at(_type).Density();
  }
}

///////////////////////////////
Material::Material(const std::string &_typename)
: dataPtr(new MaterialPrivate)
{
  // Convert to lowercase.
  std::string material = _typename;
  std::transform(material.begin(), material.end(), material.begin(), ::tolower);

  for (const std::pair<MaterialType, Material> &mat : kMaterials)
  {
    if (mat.second.Name() == material)
    {
      *this = mat.second;
    }
  }
}

///////////////////////////////
Material::Material(const double _density)
: dataPtr(new MaterialPrivate)
{
  this->dataPtr->density = _density;
}

///////////////////////////////
Material::Material(const Material &_material)
: dataPtr(new MaterialPrivate)
{
  this->dataPtr->name = _material.Name();
  this->dataPtr->density = _material.Density();
  this->dataPtr->type = _material.Type();
}

///////////////////////////////
Material::Material(Material &&_material)
{
  this->dataPtr = _material.dataPtr;
  _material.dataPtr = new MaterialPrivate;
}

///////////////////////////////
Material::~Material()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

///////////////////////////////
const std::map<MaterialType, Material> &Material::Predefined()
{
  return kMaterials;
}

///////////////////////////////
bool Material::operator==(const Material &_material) const
{
  // Not checking the name, because type should be enough.
  return this->dataPtr->type == _material.dataPtr->type &&
         equal(this->dataPtr->density, _material.dataPtr->density);
}

///////////////////////////////
bool Material::operator!=(const Material &_material) const
{
  return !(*this == _material);
}

///////////////////////////////
Material &Material::operator=(const Material &_material)
{
  this->dataPtr->name = _material.Name();
  this->dataPtr->density = _material.Density();
  this->dataPtr->type = _material.Type();
  return *this;
}

///////////////////////////////
Material &Material::operator=(Material &&_material)
{
  delete this->dataPtr;
  this->dataPtr = _material.dataPtr;
  _material.dataPtr = new MaterialPrivate;
  return *this;
}

///////////////////////////////
MaterialType Material::Type() const
{
  return this->dataPtr->type;
}

//////////////////////////////////////////////////
void Material::SetType(const MaterialType _type)
{
  this->dataPtr->type = _type;
}

//////////////////////////////////////////////////
std::string Material::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void Material::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
double Material::Density() const
{
  return this->dataPtr->density;
}

//////////////////////////////////////////////////
void Material::SetDensity(const double _density)
{
  this->dataPtr->density = _density;
}

//////////////////////////////////////////////////
void Material::SetToNearestDensity(const double _value, const double _epsilon)
{
  double min = MAX_D;
  Material result;

  for (const std::pair<MaterialType, Material> &mat : kMaterials)
  {
    double diff = std::fabs(mat.second.Density() - _value);
    if (diff < min && diff < _epsilon)
    {
      min = diff;
      result = mat.second;
    }
  }

  if (result.Type() != MaterialType::UNKNOWN_MATERIAL)
    *this = result;
}
