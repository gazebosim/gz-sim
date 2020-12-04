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
  private:
    bool rendered{false};
    common::Mesh worldMesh;
    Entity world;
    std::map<Entity, std::pair<math::Pose3d, Entity>> poses;
    std::vector<math::Matrix4d> subMeshMatrix;
    common::ColladaExporter exporter;

  private: math::Pose3d GetPose(Entity parent_, math::Pose3d currentPose)
  {
    std::stack<math::Pose3d> posesStack;
    std::pair<math::Pose3d, Entity> curr;
    math::Pose3d localPose;

    posesStack.push(currentPose);
    while (world != parent_)
    {
      curr = this->poses.at(parent_);
      posesStack.push(curr.first);
      parent_ = curr.second;
    }
    while (!posesStack.empty())
    {
      localPose = localPose * posesStack.top();
      posesStack.pop();
    }
    igndbg << ">>>>  - POSE : " << localPose <<std::endl;

    return localPose;
  }

  public: void Export(const EntityComponentManager &_ecm)
  {
    if (this->rendered) return;

    _ecm.Each<components::World, components::Name>(
      [&](const Entity & _entity,
        const components::World *,
        const components::Name * _name)->bool
    {
      world = _entity;
      this->worldMesh.SetName(_name->Data());
      return true;
    });

    _ecm.Each<components::Model, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
    {
      poses.insert(std::pair<Entity, std::pair<math::Pose3d, Entity>>(
        _entity, std::pair<math::Pose3d, Entity>(
           _pose->Data(), _parent->Data()) ));
      return true;
    });

    _ecm.Each<components::Link, components::Pose,
            components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Pose *_pose,
          const components::ParentEntity *_parent)->bool
    {
      poses.insert(std::pair<Entity, std::pair<math::Pose3d, Entity>>(
        _entity, std::pair<math::Pose3d, Entity>(
           _pose->Data(), _parent->Data()) ));
      return true;
    });


    _ecm.Each<components::Visual,
            components::Name,
            components::Pose,
            components::Geometry,
            components::Transparency,
            components::ParentEntity>(
    [&](const ignition::gazebo::Entity &_entity,
        const components::Visual *,
        const components::Name *_name,
        const components::Pose *_pose,
        const components::Geometry *_geom,
        const components::Transparency *_transparency,
        const components::ParentEntity *_parent)->bool
    {
      std::string name = _name->Data().empty() ? std::to_string(_entity) :
          _name->Data();

      igndbg << "====================== Starting: " << name << " - "
        << _entity << "  ====================== " << std::endl;

      math::Pose3d localPose = this->GetPose(_parent->Data(), _pose->Data());

      common::MaterialPtr mat(new common::Material());
      auto material = _ecm.Component<components::Material>(_entity);
      if (material != nullptr)
      {
        mat->SetDiffuse(material->Data().Diffuse());
        mat->SetAmbient(material->Data().Ambient());
        mat->SetEmissive(material->Data().Emissive());
        mat->SetSpecular(material->Data().Specular());
        mat->SetTransparency(_transparency->Data());
      }

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
          int i = this->worldMesh.AddMaterial(mat);

          subm = this->worldMesh.AddSubMesh(
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

          int i = this->worldMesh.AddMaterial(mat);

          subm = this->worldMesh.AddSubMesh(
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

          int i = this->worldMesh.AddMaterial(mat);

          subm = this->worldMesh.AddSubMesh(
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

          int i = this->worldMesh.AddMaterial(mat);

          subm = this->worldMesh.AddSubMesh(
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
              this->worldMesh.IndexOfMaterial(mesh->MaterialByIndex(
                mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get())
                << std::endl;

            int i = 0;
            if ( j != -1 )
            {
              i = this->worldMesh.IndexOfMaterial(mesh->MaterialByIndex(
                mesh->SubMeshByIndex(k).lock()->MaterialIndex()).get());
              if (i < 0)
              {
                i = this->worldMesh.AddMaterial(mesh->MaterialByIndex(
                  mesh->SubMeshByIndex(k).lock()->MaterialIndex() ));
              }
            }
            else
            {
              i = this->worldMesh.AddMaterial(mat);
            }

            igndbg << "Inserted Material index : " << i << std::endl;

            igndbg << "Texture Image : "
              << this->worldMesh.MaterialByIndex(i)->TextureImage()
              << std::endl;

            subm = this->worldMesh.AddSubMesh(
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
        ignerr << "It is NOT a mesh!"  << std::endl;
      }

      return true;
    });

    exporter.Export(&this->worldMesh, "./" + this->worldMesh.Name(), true,
                    subMeshMatrix);
    this->rendered = true;
  }
};

WorldExporter::WorldExporter() : System(),
    dataPtr(std::make_unique<WorldExporterPrivate>())
{
}

WorldExporter::~WorldExporter() = default;

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
