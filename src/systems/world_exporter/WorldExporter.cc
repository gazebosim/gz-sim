/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/Geometry.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/Material.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Transparency.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>

#include <sdf/Visual.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>

#include <ignition/common/Material.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include <ignition/math/Matrix4.hh>

#include <ignition/common/ColladaExporter.hh>

#include "WorldExporter.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;


class ignition::gazebo::systems::WorldExporterPrivate
{
  /// \brief Has the world already been exported?.
  private: bool exported{false};

  /// \brief Exports the world to a mesh.
  /// \param[_ecm] _ecm Mutable reference to the EntityComponentManager.
  public: void Export(const EntityComponentManager &_ecm)
  {
    if (this->exported) return;

    common::Mesh worldMesh;
    std::vector<math::Matrix4d> subMeshMatrix;

    _ecm.Each<components::World, components::Name>(
      [&](const Entity /*& _entity*/,
        const components::World *,
        const components::Name * _name)->bool
    {
      worldMesh.SetName(_name->Data());
      return true;
    });

    _ecm.Each<components::Visual,
            components::Name,
            components::Geometry,
            components::Transparency>(
    [&](const ignition::gazebo::Entity &_entity,
        const components::Visual *,
        const components::Name *_name,
        const components::Geometry *_geom,
        const components::Transparency *_transparency)->bool
    {
      std::string name = _name->Data().empty() ? std::to_string(_entity) :
          _name->Data();

      igndbg << "====================== Starting: " << name << " - "
        << _entity << "  ====================== " << std::endl;

      math::Pose3d localPose = gazebo::worldPose(_entity, _ecm);

      common::MaterialPtr mat = std::make_shared<common::Material>();
      auto material = _ecm.Component<components::Material>(_entity);
      if (material != nullptr)
      {
        mat->SetDiffuse(material->Data().Diffuse());
        mat->SetAmbient(material->Data().Ambient());
        mat->SetEmissive(material->Data().Emissive());
        mat->SetSpecular(material->Data().Specular());
      }
      mat->SetTransparency(_transparency->Data());

      const ignition::common::Mesh * mesh;
      std::weak_ptr<ignition::common::SubMesh> subm;
      math::Vector3d scale;
      math::Matrix4d matrix(localPose);
      ignition::common::MeshManager *meshManager =
          ignition::common::MeshManager::Instance();

      if (_geom->Data().Type() == sdf::GeometryType::BOX)
      {
        igndbg << "It is a Box!"  << std::endl;
        if (meshManager->HasMesh("unit_box"))
        {
          igndbg << "Box Found..."  << std::endl;
          mesh = meshManager->MeshByName("unit_box");
          scale = _geom->Data().BoxShape()->Size();
          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(
            *mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Box NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::CYLINDER)
      {
        if (meshManager->HasMesh("unit_cylinder"))
        {
          igndbg << "Cylinder Found"  << std::endl;
          mesh = meshManager->MeshByName("unit_cylinder");
          scale.X() = _geom->Data().CylinderShape()->Radius() * 2;
          scale.Y() = scale.X();
          scale.Z() = _geom->Data().CylinderShape()->Length();

          igndbg << "Pose : x: " << localPose << std::endl;

          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(
            *mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Cylinder NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::PLANE)
      {
        igndbg << "It is a Plane!"  << std::endl;
        if (meshManager->HasMesh("unit_plane"))
        {
          igndbg << "Plane Found"  << std::endl;
          // Create a rotation for the plane mesh to account
          // for the normal vector.
          mesh = meshManager->MeshByName("unit_plane");

          scale.X() = _geom->Data().PlaneShape()->Size().X();
          scale.Y() = _geom->Data().PlaneShape()->Size().Y();

          // // The rotation is the angle between the +z(0,0,1) vector and the
          // // normal, which are both expressed in the local (Visual) frame.
          math::Vector3d normal = _geom->Data().PlaneShape()->Normal();
          localPose.Rot().From2Axes(
            math::Vector3d::UnitZ, normal.Normalized());

          matrix = math::Matrix4d(localPose);

          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(
            *mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Plane NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::SPHERE)
      {
        igndbg << "It is a Sphere!"  << std::endl;
        if (meshManager->HasMesh("unit_sphere"))
        {
          igndbg << "Sphere Found"  << std::endl;
          mesh = meshManager->MeshByName("unit_sphere");

          scale.X() = _geom->Data().SphereShape()->Radius() * 2;
          scale.Y() = scale.X();
          scale.Z() = scale.X();

          int i = worldMesh.AddMaterial(mat);

          subm = worldMesh.AddSubMesh(
            *mesh->SubMeshByIndex(0).lock().get());
          subm.lock()->SetMaterialIndex(i);
          subm.lock()->Scale(scale);
          subMeshMatrix.push_back(matrix);
        }
        else
        {
          igndbg << "Sphere NOT Found"  << std::endl;
        }
      }
      else if (_geom->Data().Type() == sdf::GeometryType::MESH)
      {
          igndbg << "It is a mesh!"  << std::endl;

          auto fullPath = asFullPath(_geom->Data().MeshShape()->Uri(),
                                   _geom->Data().MeshShape()->FilePath());

          igndbg << "Uri : " << _geom->Data().MeshShape()->Uri() << std::endl;
          igndbg << "Path : " << _geom->Data().MeshShape()->FilePath()
            << std::endl;
          igndbg << "fullPath : " << fullPath << std::endl;

          if (fullPath.empty())
          {
            ignerr << "Mesh geometry missing uri" << std::endl;
            return true;
          }
          mesh = meshManager->Load(fullPath);

          igndbg << "Submesh count : " << mesh->SubMeshCount() << std::endl;

          if (!mesh) {
            ignerr << "MESH NOT FOUND!" << std::endl;
            return true;
          }

          for (unsigned int k = 0; k < mesh->SubMeshCount(); k++)
          {
            igndbg << ">>>>>>>>> SUBMESH : " << k << std::endl;
            int j = mesh->SubMeshByIndex(k).lock()->MaterialIndex();

            igndbg << "Query Material index : " << j << std::endl;

            igndbg << "Material : " << mesh->MaterialByIndex(
              mesh->SubMeshByIndex(k).lock()->MaterialIndex()) << std::endl;
            igndbg << "Material Ptr: " << mesh->MaterialByIndex(
              mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get()
              << std::endl;

            igndbg << "Material index on worldmesh: " <<
              worldMesh.IndexOfMaterial(mesh->MaterialByIndex(
                mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get())
                << std::endl;

            int i = 0;
            if ( j != -1 )
            {
              i = worldMesh.IndexOfMaterial(mesh->MaterialByIndex(
                mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get());
              if (i < 0)
              {
                i = worldMesh.AddMaterial(mesh->MaterialByIndex(
                  mesh->SubMeshByIndex(k).lock()->MaterialIndex() ));
              }
            }
            else
            {
              i = worldMesh.AddMaterial(mat);
            }

            igndbg << "Inserted Material index : " << i << std::endl;

            igndbg << "Texture Image : "
              << worldMesh.MaterialByIndex(i)->TextureImage()
              << std::endl;

            subm = worldMesh.AddSubMesh(
              *mesh->SubMeshByIndex(k).lock().get());
            subm.lock()->SetMaterialIndex(i);
            igndbg << "Scale : " << _geom->Data().MeshShape()->Scale()
              << std::endl;

            subm.lock()->Scale(_geom->Data().MeshShape()->Scale());
            subMeshMatrix.push_back(matrix);

            igndbg << ">>>>>>>>>" << std::endl;
          }
      }
      else
      {
        ignwarn << "Unsupported geometry type" << std::endl;
      }

      return true;
    });

    common::ColladaExporter exporter;
    exporter.Export(&worldMesh, "./" + worldMesh.Name(), true,
                    subMeshMatrix);
    this->exported = true;
  }
};

/////////////////////////////////////////////////
WorldExporter::WorldExporter() : System(),
    dataPtr(std::make_unique<WorldExporterPrivate>())
{
}

/////////////////////////////////////////////////
WorldExporter::~WorldExporter() = default;

/////////////////////////////////////////////////
void WorldExporter::PostUpdate(const UpdateInfo & /*_info*/,
    const EntityComponentManager &_ecm)
{
  this->dataPtr->Export(_ecm);
}

IGNITION_ADD_PLUGIN(WorldExporter,
                    System,
                    WorldExporter::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WorldExporter,
                          "ignition::gazebo::systems::WorldExporter")
