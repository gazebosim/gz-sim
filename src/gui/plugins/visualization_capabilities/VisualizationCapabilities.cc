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

#include "VisualizationCapabilities.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <map>
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <QQmlProperty>

#include <gz/common/Console.hh>
#include <gz/common/geospatial/Dem.hh>
#include <gz/common/geospatial/HeightmapData.hh>
#include <gz/common/geospatial/ImageHeightmap.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/StringUtils.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Uuid.hh>

#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/Helpers.hh>
#include <gz/gui/MainWindow.hh>

#include <gz/plugin/Register.hh>

#include "gz/rendering/AxisVisual.hh"
#include "gz/rendering/Capsule.hh"
#include <gz/rendering/COMVisual.hh>
#include <gz/rendering/Heightmap.hh>
#include <gz/rendering/InertiaVisual.hh>
#include <gz/rendering/JointVisual.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Text.hh>
#include <gz/rendering/WireBox.hh>

#include <gz/transport/Node.hh>

#include <sdf/Capsule.hh>
#include <sdf/Cone.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Joint.hh>
#include <sdf/Heightmap.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Polyline.hh>
#include <sdf/Root.hh>

#include "gz/sim/components/CastShadows.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Scene.hh"
#include "gz/sim/components/Transparency.hh"
#include "gz/sim/components/Visibility.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"

#include "gz/sim/Util.hh"
#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/rendering/SceneManager.hh"

namespace gz::sim
{
  class VisualizationCapabilitiesPrivate
  {
    /// \brief Update the 3D scene.
    public: void OnRender();

    /// \brief Helper function to get all child links of a model entity.
    /// \param[in] _entity Entity to find child links
    /// \return Vector of child links found for the parent entity
    public: std::vector<gz::sim::Entity>
      FindChildLinks(const gz::sim::Entity &_entity);

    /// \brief Helper function to get all children of an entity that have a
    /// pose.
    /// \param[in] _entity Entity to find children
    /// \return Vector of children found for the parent entity
    public: std::unordered_set<gz::sim::Entity>
      FindChildFrames(const gz::sim::Entity &_entity);

    /// \brief Finds the links (collision parent) that are used to create child
    /// collision visuals in RenderUtil::Update
    /// \param[in] _ecm The entity-component manager
    public: void FindCollisionLinks(const EntityComponentManager &_ecm);

    /// \brief Finds the child links for given entity from the ECM
    /// \param[in] _ecm The entity-component manager
    /// \param[in] _entity Entity to find child links
    /// \return A vector of child links found for the entity
    public: std::vector<gz::sim::Entity> FindChildLinksFromECM(
        const gz::sim::EntityComponentManager &_ecm,
        const gz::sim::Entity &_entity);

    /// \brief Finds the links (visual parent) that are used to toggle wireframe
    /// and transparent view for visuals in RenderUtil::Update
    /// \param[in] _ecm The entity-component manager
    public: void PopulateViewModeVisualLinks(
      const gz::sim::EntityComponentManager &_ecm);

    /// \brief Finds the links (inertial parent) that are used to create child
    /// inertia and center of mass visuals in RenderUtil::Update
    /// \param[in] _ecm The entity-component manager
    public: void FindInertialLinks(const EntityComponentManager &_ecm);

    /// \brief Retrieve visual based on its Gazebo entity. Note that this is
    /// different from gz-rendering's internal ID for the visual.
    /// \param[in] _entity Gazebo entity
    /// \return Pointer to requested visual, null if not found.
    public: rendering::VisualPtr VisualByEntity(Entity _entity);

    /// \brief Create a collision visual from an SDF visual element.
    /// \param[in] _id Entity which the visual corresponds to
    /// \param[in] _visual SDF describing the visual.
    /// \param[in] _parent Parent link's visual
    /// \return Pointer to created visual
    public: rendering::VisualPtr CreateCollisionVisual(
        gz::sim::Entity _id,
        const sdf::Visual &_visual,
        rendering::VisualPtr &_parent);

    /// \brief Create a geometry from an SDF element.
    /// \param[in] _geom SDF describing the geometry.
    /// \param[out] _scale Geometry's scale
    /// \param[out] _localPose Geometry's local pose
    /// \return Pointer to created geometry
    public: rendering::GeometryPtr CreateGeometry(
        const sdf::Geometry &_geom, math::Vector3d &_scale,
        math::Pose3d &_localPose);

    /// \brief Create a material from an SDF element.
    /// \param[in] _material SDF describing the material.
    /// \return Pointer to created material
    public: rendering::MaterialPtr CreateMaterial(
        const sdf::Material &_material);

    /////////////////////////////////////////////////
    // Transparent
    /////////////////////////////////////////////////
    /// \brief View an entity as transparent
    /// \param[in] _entity Entity to view as transparent
    public: void ViewTransparent(const gz::sim::Entity &_entity);

    /// \brief Callback for view as transparent request
    /// \param[in] _msg Request message to set the target to view as
    /// transparent
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewTransparent(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

    /////////////////////////////////////////////////
    // Wireframes
    /////////////////////////////////////////////////
    /// \brief View wireframes of specified entity
    /// \param[in] _entity Entity to view wireframes
    public: void ViewWireframes(const gz::sim::Entity &_entity);

    /// \brief Callback for view wireframes request
    /// \param[in] _msg Request message to set the target to view wireframes
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewWireframes(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

    /////////////////////////////////////////////////
    // Collision
    /////////////////////////////////////////////////

    /// \brief View collisions of specified entity which are shown in orange
    /// \param[in] _entity Entity to view collisions
    public: void ViewCollisions(const Entity &_entity);

    /// \brief Callback for view collisions request
    /// \param[in] _msg Request message to set the target to view collisions
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewCollisions(const msgs::StringMsg &_msg,
        msgs::Boolean &_res);

    /// \brief Create the collision visual
    /// \param[in] _id Collision entity
    /// \param[in] _collision SDF description of collision
    /// \param[in] _parent Parent link's visual
    public: rendering::VisualPtr CreateCollision(
      gz::sim::Entity _id,
      const sdf::Collision &_collision,
      gz::rendering::VisualPtr &_parent);

    /////////////////////////////////////////////////
    // COM
    /////////////////////////////////////////////////
    /// \brief View center of mass of specified entity
    /// \param[in] _entity Entity to view center of mass
    public: void ViewCOM(const Entity &_entity);

    /// \brief Callback for view center of mass request
    /// \param[in] _msg Request message to set the target to view center of
    /// mass
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewCOM(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

    /// \brief Create a center of mass visual
    /// \param[in] _id Unique visual id
    /// \param[in] _inertial Inertial component of the link
    /// \param[in] _parent Visual parent
    /// \return Visual (center of mass) object created from the inertial
    public: gz::rendering::VisualPtr CreateCOMVisual(
      gz::sim::Entity _id,
      const math::Inertiald &_inertia,
      gz::rendering::VisualPtr &_parent);

    /////////////////////////////////////////////////
    // Inertia
    /////////////////////////////////////////////////
    /// \brief View inertia of specified entity
    /// \param[in] _entity Entity to view inertia
    public: void ViewInertia(const Entity &_entity);

    /// \brief Callback for view inertia request
    /// \param[in] _msg Request message to set the target to view inertia
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewInertia(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

    /// \brief Create an inertia visual
    /// \param[in] _id Unique visual id
    /// \param[in] _inertial Inertial component of the link
    /// \param[in] _parent Visual parent
    /// \return Visual (center of mass) object created from the inertial
    public: gz::rendering::VisualPtr CreateInertiaVisual(
      gz::sim::Entity _id,
      const math::Inertiald &_inertia,
      gz::rendering::VisualPtr &_parent);

    /////////////////////////////////////////////////
    // Joints
    /////////////////////////////////////////////////

    /// \brief Callback for view joints request
    /// \param[in] _msg Request message to set the target to view center of
    /// mass
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewJoints(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

    /// \brief View joints of specified entity
    /// \param[in] _entity Entity to view joints
    public: void ViewJoints(const Entity &_entity);

    /// \brief Create a joint visual
    /// \param[in] _id Unique visual id
    /// \param[in] _joint Joint sdf dom
    /// \param[in] _childId Joint child id
    /// \param[in] _parentId Joint parent id
    /// \return Visual (joint) object created from the sdf dom
    public: rendering::VisualPtr CreateJointVisual(Entity _id,
        const sdf::Joint &_joint, Entity _childId = 0,
        Entity _parentId = 0);

    /// \brief Updates the world pose of joint parent visual
    /// according to its child.
    /// \param[in] _jointId Joint visual id.
    public: void UpdateJointParentPose(Entity _jointId);

    /////////////////////////////////////////////////
    // Frames
    /////////////////////////////////////////////////
    /// \brief View frame of specified entity
    /// \param[in] _entity Entity to view frame
    public: void ViewFrames(const Entity &_entity);

    /// \brief Callback for view frame request
    /// \param[in] _msg Request message to set the target to view frame
    /// \param[in] _res Response data
    /// \return True if the request is received
    public: bool OnViewFrames(const msgs::StringMsg &_msg,
      msgs::Boolean &_res);

    /// \brief Create a frame visual
    /// \param[in] _id Unique visual id to be used internally by gz-rendering.
    /// This is NOT a Gazebo Entity ID.
    /// \param[in] _parent Visual parent
    /// \return Visual (frame) object created
    public: rendering::VisualPtr CreateFrameVisual(
      unsigned int _id,
      rendering::VisualPtr &_parent);

    /////////////////////////////////////////////////
    /// \brief Gazebo communication node.
    public: transport::Node node;

    /// \brief Keep track of world ID, which is equivalent to the scene's
    /// root visual.
    /// Defaults to kNullEntity.
    public: Entity worldId{kNullEntity};

    /// \brief Pointer to the rendering scene
    public: gz::rendering::ScenePtr scene{nullptr};

    /// \brief Scene manager
    public: gz::sim::SceneManager sceneManager;

    /// True if the rendering component is initialized
    public: bool initialized = false;

    /// \brief A map of model entities and their corresponding children links
    public: std::map<Entity, std::vector<Entity>> modelToLinkEntities;

    /// \brief A map of model entities and their corresponding children models
    public: std::map<Entity, std::vector<Entity>> modelToModelEntities;

    /// \brief New wireframe visuals to be toggled
    public: std::vector<gz::sim::Entity> newTransparentEntities;

    /// \brief A map of link entities and their corresponding children visuals
    public: std::map<gz::sim::Entity,
      std::vector<gz::sim::Entity>> linkToVisualEntities;

    /// \brief Map of visual entity in Gazebo to visual pointers.
    public: std::map<Entity, rendering::VisualPtr> visuals;

    /////////////////////////////////////////////////
    // Transparent
    /////////////////////////////////////////////////
    /// \brief Target to view as transparent
    public: std::string viewTransparentTarget;

    /// \brief A list of links used to toggle transparent mode for visuals
    public: std::vector<Entity> newTransparentVisualLinks;

    /// \brief A map of created transparent visuals and if they are currently
    /// visible
    public: std::map<gz::sim::Entity, bool> viewingTransparent;

    /// \brief View transparent service
    public: std::string viewTransparentService;

    /////////////////////////////////////////////////
    // Wireframes
    /////////////////////////////////////////////////

    /// \brief Target to view wireframes
    public: std::string viewWireframesTarget;

    /// \brief New wireframe visuals to be toggled
    public: std::vector<Entity> newWireframes;

    /// \brief A map of created wireframe visuals and if they are currently
    /// visible
    public: std::map<Entity, bool> viewingWireframes;

    /// \brief A list of links used to toggle wireframe mode for visuals
    public: std::vector<Entity> newWireframeVisualLinks;

    /// \brief View transparent service
    public: std::string viewWireframesService;

    /////////////////////////////////////////////////
    // COM
    /////////////////////////////////////////////////

    /// \brief New center of mass visuals to be created
    public: std::vector<Entity> newCOMVisuals;

    /// \brief A list of links used to create new center of mass visuals
    public: std::vector<Entity> newCOMLinks;

    /// \brief A map of link entities and if their center of mass visuals
    /// are currently visible
    public: std::map<Entity, bool> viewingCOM;

    /// \brief A map of links and their center of mass visuals
    public: std::map<Entity, Entity> linkToCOMVisuals;

    /// \brief Target to view center of mass
    public: std::string viewCOMTarget;

    /// \brief View center of mass service
    public: std::string viewCOMService;

    /////////////////////////////////////////////////
    // Inertia
    /////////////////////////////////////////////////

    /// \brief View inertia service
    public: std::string viewInertiaService;

    /// \brief A map of entity ids and their inertials
    public: std::map<Entity, math::Inertiald> entityInertials;

    /// \brief A map of links and their inertia visuals
    public: std::map<Entity, Entity> linkToInertiaVisuals;

    /// \brief A map of link entities and if their inertias are currently
    /// visible
    public: std::map<Entity, bool> viewingInertias;

    /// \brief A list of links used to create new inertia visuals
    public: std::vector<Entity> newInertiaLinks;

    /// \brief New inertias to be created
    public: std::vector<Entity> newInertias;

    /// \brief Target to view inertia
    public: std::string viewInertiaTarget;

    /////////////////////////////////////////////////
    // Collision
    /////////////////////////////////////////////////

    /// \brief Target to view collisions
    public: std::string viewCollisionsTarget;

    /// \brief View collisions service
    public: std::string viewCollisionsService;

    /// \brief New collisions to be created
    public: std::vector<Entity> newCollisions;

    /// \brief A list of links used to create new collision visuals
    public: std::vector<Entity> newCollisionLinks;

    /// \brief A map of collision entity ids and their SDF DOM
    public: std::map<Entity, sdf::Collision> entityCollisions;

    /// \brief A map of link entities and their corresponding children
    /// collisions
    public: std::map<Entity, std::vector<Entity>> linkToCollisionEntities;

    /// \brief A map of created collision entities and if they are currently
    /// visible
    public: std::map<Entity, bool> viewingCollisions;

    /////////////////////////////////////////////////
    // Joints
    /////////////////////////////////////////////////

    /// \brief View joints service name
    public: std::string viewJointsService;

    /// \brief Target to view joints
    public: std::string viewJointsTarget;

    /// \brief New joint visuals to be created
    public: std::vector<Entity> newJoints;

    /// \brief Finds the models (joint parent) that are used to create
    /// joint visuals in RenderUtil::Update
    /// \param[in] _ecm The entity-component manager
    public: void FindJointModels(const EntityComponentManager &_ecm);

    /// \brief A list of models used to create new joint visuals
    public: std::vector<Entity> newJointModels;

    /// \brief A map of joint entity ids and their SDF DOM
    public: std::map<Entity, sdf::Joint> entityJoints;

    /// \brief A map of model entities and their corresponding children links
    public: std::map<Entity, std::vector<Entity>> modelToJointEntities;

    /// \brief A map of created joint entities and if they are currently
    /// visible
    public: std::map<Entity, bool> viewingJoints;

    /// \brief A list of joint visuals for which the parent visual poses
    /// have to be updated.
    public: std::vector<Entity> updateJointParentPoses;

    /// \brief A map of models entities and link attributes used
    /// to create joint visuals
    public: std::map<Entity, std::map<std::string, Entity>>
                             matchLinksWithEntities;

    /////////////////////////////////////////////////
    // Frame
    /////////////////////////////////////////////////

    /// \brief A list of entities that need frame visuals. Once the frame visual
    /// is created, the entity is removed from the list.
    public: std::vector<Entity> newFrameEntities;

    /// \brief Entities that have a pose. The key is the entity, the value its
    /// parent entity. Note that not all entities with pose will have frames
    /// displayed at the moment.
    public: std::unordered_map<Entity, Entity> entitiesWithPose;

    /// \brief A map of entities and whether their frame visuals
    /// are currently visible
    public: std::map<Entity, bool> viewingFrames;

    /// \brief A map of entities and the gz-rendering ID of their frame visuals
    public: std::map<Entity, unsigned int> entityToFrameVisuals;

    /// \brief Target to view frame
    public: std::string viewFramesTarget;

    /// \brief View frame service
    public: std::string viewFramesService;
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
    {
      return;
    }
    this->sceneManager.SetScene(this->scene);
  }

  // create new wireframe visuals
  for (const auto &link : this->newWireframeVisualLinks)
  {
    std::vector<Entity> visEntities =
        this->linkToVisualEntities[link];

    for (const auto &visEntity : visEntities)
    {
      if (!this->viewingWireframes[visEntity])
      {
        auto wireframeVisual = this->VisualByEntity(visEntity);
        if (wireframeVisual)
        {
          wireframeVisual->SetWireframe(true);
          this->viewingWireframes[visEntity] = true;
        }
      }
    }
  }
  this->newWireframeVisualLinks.clear();

  // update joint parent visual poses
  {
    for (const auto &jointEntity : this->updateJointParentPoses)
    {
      this->UpdateJointParentPose(jointEntity);
    }
  }
  this->updateJointParentPoses.clear();

  // create new transparent visuals
  for (const auto &link : this->newTransparentVisualLinks)
  {
    std::vector<Entity> visEntities =
        this->linkToVisualEntities[link];

    for (const auto &visEntity : visEntities)
    {
      if (!this->viewingTransparent[visEntity])
      {
        auto transparencyVisual = this->VisualByEntity(visEntity);
        if (transparencyVisual)
        {
          this->sceneManager.UpdateTransparency(transparencyVisual,
              true /* transparent */);
          this->viewingTransparent[visEntity] = true;
        }
      }
    }
  }
  this->newTransparentVisualLinks.clear();

  // create new inertia visuals
  for (const auto &link : this->newInertiaLinks)
  {
    // create a new id for the inertia visual
    auto attempts = 100000u;
    for (auto i = 0u; i < attempts; ++i)
    {
      Entity id = i;
      if (!this->scene->HasNodeId(id) && !this->scene->HasLightId(id) &&
          !this->scene->HasSensorId(id) && !this->scene->HasVisualId(id) &&
          !this->viewingInertias[link])
      {
        auto existsVisual = this->VisualByEntity(id);
        auto parentInertiaVisual = this->VisualByEntity(link);

        if (existsVisual == nullptr && parentInertiaVisual != nullptr)
        {
          this->CreateInertiaVisual(id, this->entityInertials[link],
            parentInertiaVisual);
        }
        else
        {
          continue;
        }
        this->viewingInertias[link] = true;
        this->linkToInertiaVisuals[link] = id;
        break;
      }
    }
  }
  this->newInertiaLinks.clear();

  // create new joint visuals
  {
    for (const auto &model : this->newJointModels)
    {
      std::vector<Entity> jointEntities =
          this->modelToJointEntities[model];

      for (const auto &jointEntity : jointEntities)
      {
        if (!this->scene->HasNodeId(jointEntity) &&
            !this->scene->HasLightId(jointEntity) &&
            !this->scene->HasSensorId(jointEntity) &&
            !this->scene->HasVisualId(jointEntity) &&
            !this->viewingInertias[jointEntity])
        {
          std::string childLinkName =
              this->entityJoints[jointEntity].ChildName();
          Entity childId =
              this->matchLinksWithEntities[model][childLinkName];

          std::string parentLinkName =
              this->entityJoints[jointEntity].ParentName();
          Entity parentId =
              this->matchLinksWithEntities[model][parentLinkName];

          auto joint = this->entityJoints[jointEntity];

          auto vis = this->CreateJointVisual(
              jointEntity, joint, childId, parentId);
          this->viewingJoints[jointEntity] = true;

          // Update joint parent visual pose
          if (joint.Axis(1))
          {
            this->updateJointParentPoses.push_back(jointEntity);
          }
        }
      }
    }
  }
  this->newJointModels.clear();

  // create new center of mass visuals
  for (const auto &link : this->newCOMLinks)
  {
    // create a new id for the center of mass visual
    auto attempts = 100000u;
    for (auto i = 0u; i < attempts; ++i)
    {
      Entity id = i;
      if (!this->scene->HasNodeId(id) && !this->scene->HasLightId(id) &&
          !this->scene->HasSensorId(id) && !this->scene->HasVisualId(id) &&
          !this->viewingCOM[link])
      {
        auto existsVisual = this->VisualByEntity(id);
        auto parentInertiaVisual = this->VisualByEntity(link);

        if (existsVisual == nullptr && parentInertiaVisual != nullptr)
        {
          this->CreateCOMVisual(id, this->entityInertials[link],
            parentInertiaVisual);
        }
        else
        {
          continue;
        }
        this->viewingCOM[link] = true;
        this->linkToCOMVisuals[link] = id;
        break;
      }
    }
  }
  this->newCOMLinks.clear();

  // create new collision visuals
  for (const auto &link : this->newCollisionLinks)
  {
    std::vector<Entity> colEntities =
        this->linkToCollisionEntities[link];

    for (const auto &colEntity : colEntities)
    {
      if (!this->scene->HasNodeId(colEntity) &&
          !this->scene->HasLightId(colEntity) &&
          !this->scene->HasSensorId(colEntity) &&
          !this->scene->HasVisualId(colEntity) &&
          !this->viewingCollisions[link])
      {
        auto parentCollisionVisual = this->VisualByEntity(link);
        if (parentCollisionVisual != nullptr)
        {
          auto vis = this->CreateCollision(
            colEntity,
            this->entityCollisions[colEntity],
            parentCollisionVisual);

          if (vis == nullptr)
          {
            continue;
          }
          vis->SetUserData("gui-only", static_cast<bool>(true));

          this->viewingCollisions[colEntity] = true;

          // add geometry material to originalEmissive map
          for (auto g = 0u; g < vis->GeometryCount(); ++g)
          {
            auto geom = vis->GeometryByIndex(g);

            // Geometry material
            auto geomMat = geom->Material();
            if (nullptr == geomMat)
              continue;
          }
        }
        else
        {
          continue;
        }
      }
    }
  }
  this->newCollisionLinks.clear();

  // create new frame visuals
  for (const auto &entity : this->newFrameEntities)
  {
    if (this->viewingFrames[entity])
      continue;

    auto parentVisual = this->VisualByEntity(entity);
    if (parentVisual == nullptr)
    {
      // Entities without specific visuals, like collisions and sensors,
      // aren't supported yet.
      continue;
    }

    // create a new id for the visual
    auto attempts = 100000u;
    for (Entity id = 0u; id < attempts; ++id)
    {
      if (this->scene->HasNodeId(id) || this->scene->HasLightId(id) ||
          this->scene->HasSensorId(id) || this->scene->HasVisualId(id))
      {
        continue;
      }

      this->CreateFrameVisual(id, parentVisual);
      this->viewingFrames[entity] = true;
      this->entityToFrameVisuals[entity] = id;
      break;
    }
  }
  this->newFrameEntities.clear();

  // View center of mass
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewCOM");
    if (!this->viewCOMTarget.empty())
    {
      rendering::NodePtr targetNode =
          scene->NodeByName(this->viewCOMTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewCOM(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewCOMTarget
               << "] to view center of mass" << std::endl;
      }

      this->viewCOMTarget.clear();
    }
  }

  // View inertia
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewInertia");
    if (!this->viewInertiaTarget.empty())
    {
      rendering::NodePtr targetNode =
          scene->NodeByName(this->viewInertiaTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewInertia(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewInertiaTarget
               << "] to view inertia" << std::endl;
      }

      this->viewInertiaTarget.clear();
    }
  }

  // view Transparent
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewTransparent");
    if (!this->viewTransparentTarget.empty())
    {
      rendering::NodePtr targetNode =
          this->scene->VisualByName(this->viewTransparentTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewTransparent(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewTransparentTarget
               << "] to view as transparent" << std::endl;
      }

      this->viewTransparentTarget.clear();
    }
  }

  // View collisions
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewCollisions");
    if (!this->viewCollisionsTarget.empty())
    {
      rendering::NodePtr targetNode =
          scene->NodeByName(this->viewCollisionsTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewCollisions(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewCollisionsTarget
               << "] to view collisions" << std::endl;
      }

      this->viewCollisionsTarget.clear();
    }
  }

  // View joints
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewJoints");
    if (!this->viewJointsTarget.empty())
    {
      rendering::NodePtr targetNode =
          scene->NodeByName(this->viewJointsTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewJoints(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewJointsTarget
               << "] to view joints" << std::endl;
      }

      this->viewJointsTarget.clear();
    }
  }

  // View wireframes
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewWireframes");
    if (!this->viewWireframesTarget.empty())
    {
      rendering::NodePtr targetNode =
          scene->NodeByName(this->viewWireframesTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewWireframes(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewWireframesTarget
               << "] to view wireframes" << std::endl;
      }

      this->viewWireframesTarget.clear();
    }
  }

  // View frames
  {
    GZ_PROFILE("VisualizationCapabilitiesPrivate::OnRender ViewFrames");
    if (!this->viewFramesTarget.empty())
    {
      auto targetNode = this->scene->NodeByName(this->viewFramesTarget);
      auto targetVis = std::dynamic_pointer_cast<rendering::Visual>(targetNode);

      if (targetVis && targetVis->HasUserData("gazebo-entity"))
      {
        Entity targetEntity =
            std::get<uint64_t>(targetVis->UserData("gazebo-entity"));
        this->ViewFrames(targetEntity);
      }
      else
      {
        gzerr << "Unable to find node name ["
               << this->viewFramesTarget
               << "] to view frame" << std::endl;
      }

      this->viewFramesTarget.clear();
    }
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::CreateJointVisual(
    Entity _id, const sdf::Joint &_joint,
    Entity _childId, Entity _parentId)
{
  if (!this->scene)
  {
    return rendering::VisualPtr();
  }

  if (this->visuals.find(_id) != this->visuals.end())
  {
    return rendering::VisualPtr();
  }

  rendering::VisualPtr parent;
  if (_childId != this->worldId)
  {
    parent = this->VisualByEntity(_childId);
  }

  // Name.
  std::string name = _joint.Name().empty() ? std::to_string(_id) :
    _joint.Name();
  if (parent)
  {
    name = parent->Name() +  "::" + name;
  }

  rendering::JointVisualPtr jointVisual =
    this->scene->CreateJointVisual(name);

  switch (_joint.Type())
  {
    case sdf::JointType::REVOLUTE:
      jointVisual->SetType(rendering::JointVisualType::JVT_REVOLUTE);
      break;
    case sdf::JointType::REVOLUTE2:
      jointVisual->SetType(rendering::JointVisualType::JVT_REVOLUTE2);
      break;
    case sdf::JointType::PRISMATIC:
      jointVisual->SetType(rendering::JointVisualType::JVT_PRISMATIC);
      break;
    case sdf::JointType::UNIVERSAL:
      jointVisual->SetType(rendering::JointVisualType::JVT_UNIVERSAL);
      break;
    case sdf::JointType::BALL:
      jointVisual->SetType(rendering::JointVisualType::JVT_BALL);
      break;
    case sdf::JointType::SCREW:
      jointVisual->SetType(rendering::JointVisualType::JVT_SCREW);
      break;
    case sdf::JointType::GEARBOX:
      jointVisual->SetType(rendering::JointVisualType::JVT_GEARBOX);
      break;
    case sdf::JointType::FIXED:
      jointVisual->SetType(rendering::JointVisualType::JVT_FIXED);
      break;
    default:
      jointVisual->SetType(rendering::JointVisualType::JVT_NONE);
      break;
  }

  if (parent)
  {
    jointVisual->RemoveParent();
    parent->AddChild(jointVisual);
  }

  if (_joint.Axis(1) &&
      (_joint.Type() == sdf::JointType::REVOLUTE2 ||
       _joint.Type() == sdf::JointType::UNIVERSAL
      ))
  {
    auto axis1 = _joint.Axis(0)->Xyz();
    auto axis2 = _joint.Axis(1)->Xyz();
    auto axis1UseParentFrame = _joint.Axis(0)->XyzExpressedIn() == "__model__";
    auto axis2UseParentFrame = _joint.Axis(1)->XyzExpressedIn() == "__model__";

    jointVisual->SetAxis(axis2, axis2UseParentFrame);

    auto it = this->visuals.find(_parentId);
    if (it != this->visuals.end())
    {
      auto parentName = it->second->Name();
      jointVisual->SetParentAxis(
          axis1, parentName, axis1UseParentFrame);
    }
  }
  else if (_joint.Axis(0) &&
      (_joint.Type() == sdf::JointType::REVOLUTE ||
       _joint.Type() == sdf::JointType::PRISMATIC
      ))
  {
    auto axis1 = _joint.Axis(0)->Xyz();
    auto axis1UseParentFrame = _joint.Axis(0)->XyzExpressedIn() == "__model__";

    jointVisual->SetAxis(axis1, axis1UseParentFrame);
  }
  else
  {
    // For fixed joint type, scale joint visual to the joint child link
    double childSize =
        std::max(0.1, parent->BoundingBox().Size().Length());
    auto scale = gz::math::Vector3d(childSize * 0.2,
        childSize * 0.2, childSize * 0.2);
    jointVisual->SetLocalScale(scale);
  }

  rendering::VisualPtr jointVis =
    std::dynamic_pointer_cast<rendering::Visual>(jointVisual);
  jointVis->SetUserData("gazebo-entity", _id);
  jointVis->SetUserData("pause-update", static_cast<int>(0));
  jointVis->SetUserData("gui-only", static_cast<bool>(true));
  jointVis->SetLocalPose(_joint.RawPose());
  this->visuals[_id] = jointVis;
  return jointVis;
}

////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::UpdateJointParentPose(Entity _jointId)
{
  auto visual =
      this->VisualByEntity(_jointId);

  rendering::JointVisualPtr jointVisual =
      std::dynamic_pointer_cast<rendering::JointVisual>(visual);

  auto childPose = jointVisual->WorldPose();

  if (jointVisual->ParentAxisVisual())
  {
    jointVisual->ParentAxisVisual()->SetWorldPose(childPose);

    // scale parent axis visual to the child
    auto childScale = jointVisual->LocalScale();
    jointVisual->ParentAxisVisual()->SetLocalScale(childScale);
  }
}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::CreateInertiaVisual(
  gz::sim::Entity _id,
  const math::Inertiald &_inertia,
  gz::rendering::VisualPtr &_parent)
{
  std::string name = "Inertia_" + std::to_string(_id);
  if (_parent)
    name = _parent->Name() + "::" + name;

  rendering::InertiaVisualPtr inertiaVisual =
    this->scene->CreateInertiaVisual(name);
  inertiaVisual->SetInertial(_inertia);

  rendering::VisualPtr inertiaVis =
    std::dynamic_pointer_cast<rendering::Visual>(inertiaVisual);
  inertiaVis->SetUserData("gazebo-entity", _id);
  inertiaVis->SetUserData("pause-update", static_cast<int>(0));
  inertiaVis->SetUserData("gui-only", static_cast<bool>(true));
  this->visuals[_id] = inertiaVis;
  if (_parent)
  {
    inertiaVis->RemoveParent();
    _parent->AddChild(inertiaVis);
  }
  return inertiaVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::CreateCollision(
  gz::sim::Entity _id,
  const sdf::Collision &_collision,
  rendering::VisualPtr &_parent)
{
  sdf::Material material;
  material.SetAmbient(math::Color(1.0f, 0.5088f, 0.0468f, 0.7f));
  material.SetDiffuse(math::Color(1.0f, 0.5088f, 0.0468f, 0.7f));

  sdf::Visual visual;
  visual.SetGeom(*_collision.Geom());
  visual.SetMaterial(material);
  visual.SetCastShadows(false);

  visual.SetRawPose(_collision.RawPose());
  visual.SetName(_collision.Name());

  auto collisionVis = this->CreateCollisionVisual(_id, visual, _parent);
  return collisionVis;
}

/////////////////////////////////////////////////
rendering::GeometryPtr VisualizationCapabilitiesPrivate::CreateGeometry(
  const sdf::Geometry &_geom, math::Vector3d &_scale,
  math::Pose3d &_localPose)
{
  if (!this->scene)
    return rendering::GeometryPtr();

  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose = math::Pose3d::Zero;
  rendering::GeometryPtr geom{nullptr};
  if (_geom.Type() == sdf::GeometryType::BOX)
  {
    geom = this->scene->CreateBox();
    scale = _geom.BoxShape()->Size();
  }
  else if (_geom.Type() == sdf::GeometryType::CAPSULE)
  {
    auto capsule = this->scene->CreateCapsule();
    capsule->SetRadius(_geom.CapsuleShape()->Radius());
    capsule->SetLength(_geom.CapsuleShape()->Length());
    geom = capsule;
  }
  else if (_geom.Type() == sdf::GeometryType::CONE)
  {
    geom = this->scene->CreateCone();
    scale.X() = _geom.ConeShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _geom.ConeShape()->Length();
  }
  else if (_geom.Type() == sdf::GeometryType::CYLINDER)
  {
    geom = this->scene->CreateCylinder();
    scale.X() = _geom.CylinderShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = _geom.CylinderShape()->Length();
  }
  else if (_geom.Type() == sdf::GeometryType::ELLIPSOID)
  {
    geom = this->scene->CreateSphere();
    scale.X() = _geom.EllipsoidShape()->Radii().X() * 2;
    scale.Y() = _geom.EllipsoidShape()->Radii().Y() * 2;
    scale.Z() = _geom.EllipsoidShape()->Radii().Z() * 2;
  }
  else if (_geom.Type() == sdf::GeometryType::PLANE)
  {
    geom = this->scene->CreatePlane();
    if (geom == nullptr)
      return rendering::GeometryPtr();
    scale.X() = _geom.PlaneShape()->Size().X();
    scale.Y() = _geom.PlaneShape()->Size().Y();

    // Create a rotation for the plane mesh to account for the normal vector.
    // The rotation is the angle between the +z(0,0,1) vector and the
    // normal, which are both expressed in the local (Visual) frame.
    math::Vector3d normal = _geom.PlaneShape()->Normal();
    localPose.Rot().SetFrom2Axes(math::Vector3d::UnitZ, normal.Normalized());
  }
  else if (_geom.Type() == sdf::GeometryType::SPHERE)
  {
    geom = this->scene->CreateSphere();
    scale.X() = _geom.SphereShape()->Radius() * 2;
    scale.Y() = scale.X();
    scale.Z() = scale.X();
  }
  else if (_geom.Type() == sdf::GeometryType::MESH)
  {
    auto fullPath = asFullPath(_geom.MeshShape()->Uri(),
        _geom.MeshShape()->FilePath());
    if (fullPath.empty())
    {
      gzerr << "Mesh geometry missing uri" << std::endl;
      return geom;
    }
    rendering::MeshDescriptor descriptor;

    // Assume absolute path to mesh file
    descriptor.meshName = fullPath;
    gz::common::MeshManager *meshManager =
        gz::common::MeshManager::Instance();
    descriptor.mesh = meshManager->Load(descriptor.meshName);
    if (descriptor.mesh)
    {
      if (_geom.MeshShape()->Optimization() != sdf::MeshOptimization::NONE)
      {
        const common::Mesh *optimizedMesh =
            optimizeMesh(*_geom.MeshShape(), *descriptor.mesh);
        if (optimizedMesh)
        {
          descriptor.mesh = optimizedMesh;
          // if submesh is requested, it should be handled in the optimizeMesh
          // function so we do not need need to pass these flags to
          // gz-rendering
          descriptor.subMeshName = "";
          descriptor.centerSubMesh = false;
        }
      }
      geom = this->scene->CreateMesh(descriptor);
    }
    else
    {
      gzerr << "Failed to load mesh: " << descriptor.meshName << std::endl;
    }
    scale = _geom.MeshShape()->Scale();
  }
  else if (_geom.Type() == sdf::GeometryType::HEIGHTMAP)
  {
    auto fullPath = asFullPath(_geom.HeightmapShape()->Uri(),
        _geom.HeightmapShape()->FilePath());
    if (fullPath.empty())
    {
      gzerr << "Heightmap geometry missing URI" << std::endl;
      return geom;
    }

    std::shared_ptr<common::HeightmapData> data;
    std::string lowerFullPath = common::lowercase(fullPath);
    // check if heightmap is an image
    if (common::EndsWith(lowerFullPath, ".png")
        || common::EndsWith(lowerFullPath, ".jpg")
        || common::EndsWith(lowerFullPath, ".jpeg"))
    {
      auto img = std::make_shared<common::ImageHeightmap>();
      if (img->Load(fullPath) < 0)
      {
        gzerr << "Failed to load heightmap image data from ["
               << fullPath << "]" << std::endl;
        return geom;
      }
      data = img;
    }
    // DEM
    else
    {
      auto dem = std::make_shared<common::Dem>();
      if (dem->Load(fullPath) < 0)
      {
        gzerr << "Failed to load heightmap dem data from ["
               << fullPath << "]" << std::endl;
        return geom;
      }
      data = dem;
    }

    rendering::HeightmapDescriptor descriptor;
    descriptor.SetData(data);
    descriptor.SetSize(_geom.HeightmapShape()->Size());
    descriptor.SetSampling(_geom.HeightmapShape()->Sampling());

    for (uint64_t i = 0; i < _geom.HeightmapShape()->TextureCount(); ++i)
    {
      auto textureSdf = _geom.HeightmapShape()->TextureByIndex(i);
      rendering::HeightmapTexture textureDesc;
      textureDesc.SetSize(textureSdf->Size());
      textureDesc.SetDiffuse(asFullPath(textureSdf->Diffuse(),
          _geom.HeightmapShape()->FilePath()));
      textureDesc.SetNormal(asFullPath(textureSdf->Normal(),
          _geom.HeightmapShape()->FilePath()));
      descriptor.AddTexture(textureDesc);
    }

    for (uint64_t i = 0; i < _geom.HeightmapShape()->BlendCount(); ++i)
    {
      auto blendSdf = _geom.HeightmapShape()->BlendByIndex(i);
      rendering::HeightmapBlend blendDesc;
      blendDesc.SetMinHeight(blendSdf->MinHeight());
      blendDesc.SetFadeDistance(blendSdf->FadeDistance());
      descriptor.AddBlend(blendDesc);
    }

    geom = this->scene->CreateHeightmap(descriptor);
    if (nullptr == geom)
    {
      gzerr << "Failed to create heightmap [" << fullPath << "]" << std::endl;
    }
    scale = _geom.HeightmapShape()->Size();
  }
  else if (_geom.Type() == sdf::GeometryType::POLYLINE)
  {
    std::vector<std::vector<math::Vector2d>> vertices;
    for (const auto &polyline : _geom.PolylineShape())
    {
      vertices.push_back(polyline.Points());
    }

    std::string name("POLYLINE_" + common::Uuid().String());

    auto meshManager = common::MeshManager::Instance();
    meshManager->CreateExtrudedPolyline(name, vertices,
        _geom.PolylineShape()[0].Height());

    rendering::MeshDescriptor descriptor;
    descriptor.meshName = name;
    descriptor.mesh = meshManager->MeshByName(name);

    geom = this->scene->CreateMesh(descriptor);
  }
  else
  {
    gzerr << "Unsupported geometry type" << std::endl;
  }
  _scale = scale;
  _localPose = localPose;
  return geom;
}

/////////////////////////////////////////////////
rendering::MaterialPtr VisualizationCapabilitiesPrivate::CreateMaterial(
    const sdf::Material &_material)
{
  if (!this->scene)
    return rendering::MaterialPtr();

  rendering::MaterialPtr material = this->scene->CreateMaterial();
  material->SetAmbient(_material.Ambient());
  material->SetDiffuse(_material.Diffuse());
  material->SetSpecular(_material.Specular());
  material->SetEmissive(_material.Emissive());
  material->SetRenderOrder(_material.RenderOrder());

  // parse PBR params
  const sdf::Pbr *pbr = _material.PbrMaterial();
  if (pbr)
  {
    sdf::PbrWorkflow *workflow = nullptr;
    const sdf::PbrWorkflow *metal =
        pbr->Workflow(sdf::PbrWorkflowType::METAL);
    if (metal)
    {
      double roughness = metal->Roughness();
      double metalness = metal->Metalness();
      material->SetRoughness(roughness);
      material->SetMetalness(metalness);

      // roughness map
      std::string roughnessMap = metal->RoughnessMap();
      if (!roughnessMap.empty())
      {
        std::string fullPath = common::findFile(
            asFullPath(roughnessMap, _material.FilePath()));
        if (!fullPath.empty())
          material->SetRoughnessMap(fullPath);
        else
          gzerr << "Unable to find file [" << roughnessMap << "]\n";
      }

      // metalness map
      std::string metalnessMap = metal->MetalnessMap();
      if (!metalnessMap.empty())
      {
        std::string fullPath = common::findFile(
            asFullPath(metalnessMap, _material.FilePath()));
        if (!fullPath.empty())
          material->SetMetalnessMap(fullPath);
        else
          gzerr << "Unable to find file [" << metalnessMap << "]\n";
      }
      workflow = const_cast<sdf::PbrWorkflow *>(metal);
    }
    else
    {
      gzerr << "PBR material: currently only metal workflow is supported"
             << std::endl;
    }

    // albedo map
    std::string albedoMap = workflow->AlbedoMap();
    if (!albedoMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(albedoMap, _material.FilePath()));
      if (!fullPath.empty())
      {
        material->SetTexture(fullPath);
        // Use alpha channel for transparency
        material->SetAlphaFromTexture(true, 0.5, _material.DoubleSided());
      }
      else
        gzerr << "Unable to find file [" << albedoMap << "]\n";
    }

    // normal map
    std::string normalMap = workflow->NormalMap();
    if (!normalMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(normalMap, _material.FilePath()));
      if (!fullPath.empty())
        material->SetNormalMap(fullPath);
      else
        gzerr << "Unable to find file [" << normalMap << "]\n";
    }


    // environment map
    std::string environmentMap = workflow->EnvironmentMap();
    if (!environmentMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(environmentMap, _material.FilePath()));
      if (!fullPath.empty())
        material->SetEnvironmentMap(fullPath);
      else
        gzerr << "Unable to find file [" << environmentMap << "]\n";
    }

    // emissive map
    std::string emissiveMap = workflow->EmissiveMap();
    if (!emissiveMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(emissiveMap, _material.FilePath()));
      if (!fullPath.empty())
        material->SetEmissiveMap(fullPath);
      else
        gzerr << "Unable to find file [" << emissiveMap << "]\n";
    }

    // light map
    std::string lightMap = workflow->LightMap();
    if (!lightMap.empty())
    {
      std::string fullPath = common::findFile(
          asFullPath(lightMap, _material.FilePath()));
      if (!fullPath.empty())
      {
        unsigned int uvSet = workflow->LightMapTexCoordSet();
        material->SetLightMap(fullPath, uvSet);
      }
      else
      {
        gzerr << "Unable to find file [" << lightMap << "]\n";
      }
    }
  }
  return material;
}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::CreateCollisionVisual(
  gz::sim::Entity _id,
  const sdf::Visual &_visual,
  rendering::VisualPtr &_parent)
{
  if (!this->scene)
    return rendering::VisualPtr();

  if (this->visuals.find(_id) != this->visuals.end())
  {
    return rendering::VisualPtr();
  }

  if (!_visual.Geom())
    return rendering::VisualPtr();

  if (_visual.Geom()->Type() == sdf::GeometryType::HEIGHTMAP)
  {
    gzwarn << "Collision visualization for heightmaps are not supported yet."
           << std::endl;
    return rendering::VisualPtr();
  }

  std::string name = _visual.Name().empty() ? std::to_string(_id) :
      _visual.Name();
  if (_parent)
    name = _parent->Name() + "::" + name;
  if (this->scene->HasVisualName(name))
  {
    auto vis = this->scene->VisualByName(name);
    this->visuals[_id] = vis;
    return vis;
  }
  rendering::VisualPtr visualVis = this->scene->CreateVisual(name);
  visualVis->SetUserData("gazebo-entity", _id);
  visualVis->SetUserData("pause-update", static_cast<int>(0));
  visualVis->SetLocalPose(_visual.RawPose());

  math::Vector3d scale = math::Vector3d::One;
  math::Pose3d localPose;
  rendering::GeometryPtr geom =
      this->CreateGeometry(*_visual.Geom(), scale, localPose);

  if (geom)
  {
    /// localPose is currently used to handle the normal vector in plane visuals
    /// In general, this can be used to store any local transforms between the
    /// parent Visual and geometry.
    if (localPose != math::Pose3d::Zero)
    {
      rendering::VisualPtr geomVis =
          this->scene->CreateVisual(name + "_geom");
      geomVis->AddGeometry(geom);
      geomVis->SetLocalPose(localPose);
      visualVis->AddChild(geomVis);
    }
    else
    {
      visualVis->AddGeometry(geom);
    }

    visualVis->SetLocalScale(scale);

    // set material
    rendering::MaterialPtr material{nullptr};
    if (_visual.Geom()->Type() == sdf::GeometryType::HEIGHTMAP)
    {
      // Heightmap's material is loaded together with it.
    }
    else if (_visual.Material())
    {
      material = this->CreateMaterial(*_visual.Material());
    }
    // Don't set a default material for meshes because they
    // may have their own
    // TODO(anyone) support overriding mesh material
    else if (_visual.Geom()->Type() != sdf::GeometryType::MESH)
    {
      // create default material
      material = this->scene->Material("gz-grey");
      if (!material)
      {
        material = this->scene->CreateMaterial("gz-grey");
        material->SetAmbient(0.3, 0.3, 0.3);
        material->SetDiffuse(0.7, 0.7, 0.7);
        material->SetSpecular(1.0, 1.0, 1.0);
        material->SetRoughness(0.2f);
        material->SetMetalness(1.0f);
      }
    }
    else
    {
      // meshes created by mesh loader may have their own materials
      // update/override their properties based on input sdf element values
      auto mesh = std::dynamic_pointer_cast<rendering::Mesh>(geom);
      for (unsigned int i = 0; i < mesh->SubMeshCount(); ++i)
      {
        auto submesh = mesh->SubMeshByIndex(i);
        auto submeshMat = submesh->Material();
        if (submeshMat)
        {
          double productAlpha = (1.0-_visual.Transparency()) *
              (1.0 - submeshMat->Transparency());
          submeshMat->SetTransparency(1 - productAlpha);
          submeshMat->SetCastShadows(_visual.CastShadows());
        }
      }
    }

    if (material)
    {
      // set transparency
      material->SetTransparency(_visual.Transparency());

      // cast shadows
      material->SetCastShadows(_visual.CastShadows());

      geom->SetMaterial(material);
      // todo(anyone) SetMaterial function clones the input material.
      // but does not take ownership of it so we need to destroy it here.
      // This is not ideal. We should let gz-rendering handle the lifetime
      // of this material
      this->scene->DestroyMaterial(material);
    }
  }
  else
  {
    gzerr << "Failed to load geometry for visual: " << _visual.Name()
           << std::endl;
  }

  // visibility flags
  visualVis->SetVisibilityFlags(_visual.VisibilityFlags());

  this->visuals[_id] = visualVis;
  if (_parent)
    _parent->AddChild(visualVis);

  return visualVis;

}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::CreateCOMVisual(
  gz::sim::Entity _id,
  const math::Inertiald &_inertia,
  rendering::VisualPtr &_parent)
{
  std::string name = "COM_" + std::to_string(_id);
  if (_parent)
    name = _parent->Name() + "::" + name;

  rendering::COMVisualPtr comVisual =
    this->scene->CreateCOMVisual(name);
  comVisual->SetInertial(_inertia);

  rendering::VisualPtr comVis =
    std::dynamic_pointer_cast<rendering::Visual>(comVisual);
  comVis->SetUserData("gazebo-entity", _id);
  comVis->SetUserData("pause-update", static_cast<int>(0));
  comVis->SetUserData("gui-only", static_cast<bool>(true));
  this->visuals[_id] = comVis;

  if (_parent)
  {
    comVis->RemoveParent();
    _parent->AddChild(comVis);
  }

  return comVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::CreateFrameVisual(
  unsigned int _id, rendering::VisualPtr &_parent)
{
  std::string name = "Frame_" + std::to_string(_id);
  if (_parent)
    name = _parent->Name() + "::" + name;

  auto frameVisual = this->scene->CreateAxisVisual(_id, name);

  auto frameVis = std::dynamic_pointer_cast<rendering::Visual>(frameVisual);
  frameVis->SetUserData("pause-update", static_cast<int>(0));

  // Scale w.r.t. parent
  double parentSize = std::max(0.1, _parent->BoundingBox().Size().Length());
  auto scale = parentSize * 0.2;
  frameVis->SetInheritScale(false);
  frameVis->SetLocalScale(scale);

  // Add frame name
  auto textGeom = this->scene->CreateText();
  // Ogre 2 doesn't support Text, see
  // https://github.com/gazebosim/gz-rendering/issues/487
  if (nullptr != textGeom)
  {
    textGeom->SetFontName("Liberation Sans");
    textGeom->SetTextString(_parent->Name());
    textGeom->SetShowOnTop(true);
    textGeom->SetTextAlignment(rendering::TextHorizontalAlign::CENTER,
                               rendering::TextVerticalAlign::BOTTOM);
    auto textVis = this->scene->CreateVisual();
    textVis->AddGeometry(textGeom);
    textVis->SetLocalPosition(0, 0, scale * 0.5);
    textVis->SetLocalScale(scale * 0.5);

    frameVis->AddChild(textVis);
  }

  if (_parent)
  {
    frameVis->RemoveParent();
    _parent->AddChild(frameVis);
  }

  return frameVis;
}

/////////////////////////////////////////////////
rendering::VisualPtr VisualizationCapabilitiesPrivate::VisualByEntity(
  Entity _entity)
{
  for (unsigned int i = 0; i < this->scene->VisualCount(); ++i)
  {
    auto visual = this->scene->VisualByIndex(i);

    try
    {
      Entity visualEntity = std::get<uint64_t>(
        visual->UserData("gazebo-entity"));

      if (visualEntity == _entity)
      {
        return visual;
      }
    }
    catch (std::bad_variant_access &)
    {
      // It's ok to get here
    }
  }
  return nullptr;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewTransparent(
  const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewTransparentTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewWireframes(
  const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewWireframesTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewCOM(
  const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewCOMTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewJoints(
  const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewJointsTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewInertia(
  const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewInertiaTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewCollisions(
  const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewCollisionsTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
bool VisualizationCapabilitiesPrivate::OnViewFrames(
    const msgs::StringMsg &_msg, msgs::Boolean &_res)
{
  this->viewFramesTarget = _msg.data();

  _res.set_data(true);
  return true;
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewCollisions(const Entity &_entity)
{
  std::vector<Entity> colEntities;

  if (this->linkToCollisionEntities.find(_entity) !=
      this->linkToCollisionEntities.end())
  {
    colEntities = this->linkToCollisionEntities[_entity];
  }

  // Find all existing child links for this entity
  std::vector<Entity> links = this->FindChildLinks(_entity);

  for (const auto &link : links)
  {
    colEntities.insert(colEntities.end(),
        this->linkToCollisionEntities[link].begin(),
        this->linkToCollisionEntities[link].end());
  }

  // create and/or toggle collision visuals
  bool showCol, showColInit = false;

  // first loop looks for new collisions
  for (const auto &colEntity : colEntities)
  {
    if (this->viewingCollisions.find(colEntity) ==
        this->viewingCollisions.end())
    {
      this->newCollisions.push_back(_entity);
      showColInit = showCol = true;
    }
  }

  // second loop toggles already created collisions
  for (const auto &colEntity : colEntities)
  {
    if (this->viewingCollisions.find(colEntity) ==
        this->viewingCollisions.end())
      continue;

    // when viewing multiple collisions (e.g. _entity is a model),
    // boolean for view collisions is based on first colEntity in list
    if (!showColInit)
    {
      showCol = !this->viewingCollisions[colEntity];
      showColInit = true;
    }

    auto colVisual = this->VisualByEntity(colEntity);
    if (colVisual)
    {
      this->viewingCollisions[colEntity] = showCol;
      colVisual->SetVisible(showCol);
    }
  }
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewInertia(const Entity &_entity)
{
  std::vector<Entity> inertiaLinks = this->FindChildLinks(_entity);

  // check if _entity has an inertial component (_entity is a link)
  if (this->entityInertials.find(_entity) !=
      this->entityInertials.end())
    inertiaLinks.push_back(_entity);

  // create and/or toggle inertia visuals
  bool showInertia, showInertiaInit = false;
  // first loop looks for new inertias
  for (const auto &inertiaLink : inertiaLinks)
  {
    if (this->viewingInertias.find(inertiaLink) ==
        this->viewingInertias.end())
    {
      this->newInertias.push_back(_entity);
      showInertiaInit = showInertia = true;
    }
  }

  // second loop toggles already created inertias
  for (const auto &inertiaLink : inertiaLinks)
  {
    if (this->viewingInertias.find(inertiaLink) ==
        this->viewingInertias.end())
      continue;

    // when viewing multiple inertias (e.g. _entity is a model),
    // boolean for view inertias is based on first inrEntity in list
    if (!showInertiaInit)
    {
      showInertia = !this->viewingInertias[inertiaLink];
      showInertiaInit = true;
    }

    Entity inertiaVisualId = this->linkToInertiaVisuals[inertiaLink];

    auto inertiaVisual = this->VisualByEntity(inertiaVisualId);
    if (inertiaVisual)
    {
      this->viewingInertias[inertiaLink] = showInertia;
      inertiaVisual->SetVisible(showInertia);
    }
  }
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewJoints(const Entity &_entity)
{
  std::vector<Entity> jointEntities;
  if (this->modelToJointEntities.find(_entity) !=
           this->modelToJointEntities.end())
  {
    jointEntities.insert(jointEntities.end(),
        this->modelToJointEntities[_entity].begin(),
        this->modelToJointEntities[_entity].end());
  }

  if (this->modelToModelEntities.find(_entity) !=
      this->modelToModelEntities.end())
  {
    std::stack<Entity> modelStack;
    modelStack.push(_entity);

    std::vector<Entity> childModels;
    while (!modelStack.empty())
    {
      Entity model = modelStack.top();
      modelStack.pop();

      jointEntities.insert(jointEntities.end(),
          this->modelToJointEntities[model].begin(),
          this->modelToJointEntities[model].end());

      childModels = this->modelToModelEntities[model];
      for (const auto &childModel : childModels)
      {
        modelStack.push(childModel);
      }
    }
  }

  // Toggle joints
  bool showJoint, showJointInit = false;

  // first loop looks for new joints
  for (const auto &jointEntity : jointEntities)
  {
    if (this->viewingJoints.find(jointEntity) ==
        this->viewingJoints.end())
    {
      this->newJoints.push_back(_entity);
      showJointInit = showJoint = true;
    }
  }

  // second loop toggles joints
  for (const auto &jointEntity : jointEntities)
  {
    if (this->viewingJoints.find(jointEntity) ==
        this->viewingJoints.end())
      continue;

    // when viewing multiple joints (e.g. _entity is a model),
    // boolean for view joints is based on first jointEntity in list
    if (!showJointInit)
    {
      showJoint = !this->viewingJoints[jointEntity];
      showJointInit = true;
    }

    rendering::VisualPtr jointVisual =
        this->VisualByEntity(jointEntity);
    if (jointVisual == nullptr)
    {
      gzerr << "Could not find visual for entity [" << jointEntity
             << "]" << std::endl;
      continue;
    }

    this->viewingJoints[jointEntity] = showJoint;
    jointVisual->SetVisible(showJoint);
  }
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewCOM(const Entity &_entity)
{
  std::vector<Entity> inertiaLinks = this->FindChildLinks(_entity);

  // check if _entity has an inertial component (_entity is a link)
  if (this->entityInertials.find(_entity) !=
      this->entityInertials.end())
    inertiaLinks.push_back(_entity);

  // create and/or toggle center of mass visuals
  bool showCOM, showCOMInit = false;
  // first loop looks for new center of mass visuals
  for (const auto &inertiaLink : inertiaLinks)
  {
    if (this->viewingCOM.find(inertiaLink) ==
        this->viewingCOM.end())
    {
      this->newCOMVisuals.push_back(_entity);
      showCOMInit = showCOM = true;
    }
  }

  // second loop toggles already created center of mass visuals
  for (const auto &inertiaLink : inertiaLinks)
  {
    if (this->viewingCOM.find(inertiaLink) ==
        this->viewingCOM.end())
      continue;

    // when viewing multiple center of mass visuals (e.g. _entity is a model),
    // boolean for view center of mass is based on first inertiaEntity in list
    if (!showCOMInit)
    {
      showCOM = !this->viewingCOM[inertiaLink];
      showCOMInit = true;
    }

    Entity comVisualId = this->linkToCOMVisuals[inertiaLink];

    auto comVisual = this->VisualByEntity(comVisualId);
    if (comVisual)
    {
      this->viewingCOM[inertiaLink] = showCOM;
      comVisual->SetVisible(showCOM);
    }
  }
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewWireframes(const Entity &_entity)
{
  std::vector<Entity> visEntities;

  if (this->linkToVisualEntities.find(_entity) !=
      this->linkToVisualEntities.end())
  {
    visEntities = this->linkToVisualEntities[_entity];
  }

  // Find all existing child links for this entity
  std::vector<Entity> links = this->FindChildLinks(_entity);

  for (const auto &link : links)
  {
    visEntities.insert(visEntities.end(),
        this->linkToVisualEntities[link].begin(),
        this->linkToVisualEntities[link].end());
  }

  // Toggle wireframes
  bool showWireframe, showWireframeInit = false;

  // first loop looks for new wireframes
  for (const auto &visEntity : visEntities)
  {
    if (this->viewingWireframes.find(visEntity) ==
        this->viewingWireframes.end())
    {
      this->newWireframes.push_back(_entity);
      showWireframeInit = showWireframe = true;
    }
  }

  // second loop toggles wireframes
  for (const auto &visEntity : visEntities)
  {
    if (this->viewingWireframes.find(visEntity) ==
        this->viewingWireframes.end())
      continue;

    // when viewing multiple wireframes (e.g. _entity is a model),
    // boolean for view wireframe is based on first visEntity in list
    if (!showWireframeInit)
    {
      showWireframe = !this->viewingWireframes[visEntity];
      showWireframeInit = true;
    }

    auto wireframesVisual = this->VisualByEntity(visEntity);
    if (wireframesVisual)
    {
      this->viewingWireframes[visEntity] = showWireframe;
      wireframesVisual->SetWireframe(showWireframe);
    }
  }
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewFrames(const Entity &_entity)
{
  // Show if currently hidden and vice-versa
  bool showFrames = (this->viewingFrames.find(_entity) ==
        this->viewingFrames.end()) || !this->viewingFrames[_entity];

  auto descendants = this->FindChildFrames(_entity);

  for (const auto &descendant : descendants)
  {
    // Add new descendants to newFrameEntities so their visuals are created in
    // the next render callback
    if (this->viewingFrames.find(descendant) == this->viewingFrames.end())
    {
      this->newFrameEntities.push_back(descendant);
      continue;
    }

    auto frameVisualId = this->entityToFrameVisuals[descendant];

    auto frameVisual = this->scene->VisualById(frameVisualId);
    if (frameVisual == nullptr)
    {
      gzerr << "Failed to find frame visual with ID [" << frameVisualId
             << "] for entity [" << descendant << "]" << std::endl;
      continue;
    }

    this->viewingFrames[descendant] = showFrames;
    frameVisual->SetVisible(showFrames);
  }
}

/////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::ViewTransparent(const Entity &_entity)
{
  std::vector<Entity> visEntities;

  if (this->linkToVisualEntities.find(_entity) !=
      this->linkToVisualEntities.end())
  {
    visEntities = this->linkToVisualEntities[_entity];
  }

  // Find all existing child links for this entity
  std::vector<Entity> links = this->FindChildLinks(_entity);

  for (const auto &link : links)
  {
    visEntities.insert(visEntities.end(),
        this->linkToVisualEntities[link].begin(),
        this->linkToVisualEntities[link].end());
  }

  // Toggle transparent mode
  bool showTransparent, showTransparentInit = false;

  // first loop looks for new transparent entities
  for (const auto &visEntity : visEntities)
  {
    if (this->viewingTransparent.find(visEntity) ==
        this->viewingTransparent.end())
    {
      this->newTransparentEntities.push_back(_entity);
      showTransparentInit = showTransparent = true;
    }
  }

  // second loop toggles transparent mode
  for (const auto &visEntity : visEntities)
  {
    if (this->viewingTransparent.find(visEntity) ==
        this->viewingTransparent.end())
      continue;

    // when viewing multiple transparent visuals (e.g. _entity is a model),
    // boolean for view as transparent is based on first visEntity in list
    if (!showTransparentInit)
    {
      showTransparent = !this->viewingTransparent[visEntity];
      showTransparentInit = true;
    }

    auto transparentVisual = this->VisualByEntity(visEntity);
    if (transparentVisual)
    {
      this->viewingTransparent[visEntity] = showTransparent;

      this->sceneManager.UpdateTransparency(transparentVisual,
                showTransparent);
    }
  }
}

/////////////////////////////////////////////////
std::vector<Entity> VisualizationCapabilitiesPrivate::FindChildLinks(
  const Entity &_entity)
{
  std::vector<Entity> links;

  if (this->modelToLinkEntities.find(_entity) !=
           this->modelToLinkEntities.end())
  {
    links.insert(links.end(),
        this->modelToLinkEntities[_entity].begin(),
        this->modelToLinkEntities[_entity].end());
  }

  if (this->modelToModelEntities.find(_entity) !=
      this->modelToModelEntities.end())
  {
    std::stack<Entity> modelStack;
    modelStack.push(_entity);

    std::vector<Entity> childModels;
    while (!modelStack.empty())
    {
      Entity model = modelStack.top();
      modelStack.pop();

      links.insert(links.end(),
          this->modelToLinkEntities[model].begin(),
          this->modelToLinkEntities[model].end());

      childModels = this->modelToModelEntities[model];
      for (const auto &childModel : childModels)
      {
        modelStack.push(childModel);
      }
    }
  }

  return links;
}

/////////////////////////////////////////////////
std::unordered_set<Entity> VisualizationCapabilitiesPrivate::FindChildFrames(
  const Entity &_entity)
{
  std::unordered_set<Entity> descendants;

  // Display own frame
  if (this->entitiesWithPose.find(_entity) != this->entitiesWithPose.end())
  {
    descendants.insert(_entity);
  }

  // Recursively add descendants
  for (auto entityAndParent : this->entitiesWithPose)
  {
    // Not my child
    if (entityAndParent.second != _entity)
    {
      continue;
    }

    auto grandChildren = this->FindChildFrames(entityAndParent.first);
    descendants.insert(grandChildren.begin(), grandChildren.end());
  }

  return descendants;
}

//////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::FindJointModels(
  const EntityComponentManager &_ecm)
{
  if (this->newJoints.empty())
  {
    return;
  }

  for (const auto &entity : this->newJoints)
  {
    std::vector<Entity> models;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}))
    {
      std::stack<Entity> modelStack;
      modelStack.push(entity);

      std::vector<Entity> childModels;
      while (!modelStack.empty())
      {
        Entity model = modelStack.top();
        modelStack.pop();
        models.push_back(model);

        childModels =
            _ecm.EntitiesByComponents(components::ParentEntity(model),
                                      components::Model());
        for (const auto &childModel : childModels)
        {
            modelStack.push(childModel);
        }
      }
    }
    else
    {
      gzerr << "Entity [" << entity
             << "] for viewing joints must be a model"
             << std::endl;
      continue;
    }

    this->newJointModels.insert(this->newJointModels.end(),
        models.begin(),
        models.end());
  }
  this->newJoints.clear();
}

//////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::FindInertialLinks(
  const EntityComponentManager &_ecm)
{
  for (const auto &entity : this->newInertias)
  {
    std::vector<Entity> links;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}) ||
        _ecm.EntityMatches(entity,
                std::set<ComponentTypeId>{components::Link::typeId}))
    {
      links = this->FindChildLinksFromECM(_ecm, entity);
    }
    else
    {
      gzerr << "Entity [" << entity
             << "] for viewing inertia must be a model or link"
             << std::endl;
      continue;
    }

    this->newInertiaLinks.insert(this->newInertiaLinks.end(),
        links.begin(),
        links.end());
  }
  this->newInertias.clear();

  for (const auto &entity : this->newCOMVisuals)
  {
    std::vector<Entity> links;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}) ||
        _ecm.EntityMatches(entity,
                std::set<ComponentTypeId>{components::Link::typeId}))
    {
      links = this->FindChildLinksFromECM(_ecm, entity);
    }
    else
    {
      gzerr << "Entity [" << entity
             << "] for viewing center of mass must be a model or link"
             << std::endl;
      continue;
    }

    this->newCOMLinks.insert(this->newCOMLinks.end(),
        links.begin(),
        links.end());
  }
  this->newCOMVisuals.clear();
}

//////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::FindCollisionLinks(
  const EntityComponentManager &_ecm)
{
  if (this->newCollisions.empty())
    return;

  for (const auto &entity : this->newCollisions)
  {
    std::vector<Entity> links;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}) ||
        _ecm.EntityMatches(entity,
                std::set<ComponentTypeId>{components::Link::typeId}))
    {
      links = this->FindChildLinksFromECM(_ecm, entity);
    }
    else
    {
      gzerr << "Entity [" << entity
             << "] for viewing collision must be a model or link"
             << std::endl;
      continue;
    }

    this->newCollisionLinks.insert(this->newCollisionLinks.end(),
        links.begin(),
        links.end());
  }
  this->newCollisions.clear();
}

//////////////////////////////////////////////////
void VisualizationCapabilitiesPrivate::PopulateViewModeVisualLinks(
                        const EntityComponentManager &_ecm)
{
  // Find links to toggle wireframes
  for (const auto &entity : this->newWireframes)
  {
    std::vector<Entity> links;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}) ||
        _ecm.EntityMatches(entity,
                std::set<ComponentTypeId>{components::Link::typeId}))
    {
      links = this->FindChildLinksFromECM(_ecm, entity);
    }
    else
    {
      gzerr << "Entity [" << entity
             << "] for viewing wireframe must be a model or link"
             << std::endl;
      continue;
    }

    this->newWireframeVisualLinks.insert(this->newWireframeVisualLinks.end(),
        links.begin(),
        links.end());
  }
  this->newWireframes.clear();

  // Find links to view as transparent
  for (const auto &entity : this->newTransparentEntities)
  {
    std::vector<Entity> links;
    if (_ecm.EntityMatches(entity,
          std::set<ComponentTypeId>{components::Model::typeId}) ||
        _ecm.EntityMatches(entity,
                std::set<ComponentTypeId>{components::Link::typeId}))
    {
      links = this->FindChildLinksFromECM(_ecm, entity);
    }
    else
    {
      gzerr << "Entity [" << entity
             << "] for viewing as transparent must be a model or link"
             << std::endl;
      continue;
    }

    this->newTransparentVisualLinks.insert(
        this->newTransparentVisualLinks.end(),
        links.begin(),
        links.end());
  }
  this->newTransparentEntities.clear();
}

//////////////////////////////////////////////////
std::vector<Entity> VisualizationCapabilitiesPrivate::FindChildLinksFromECM(
    const EntityComponentManager &_ecm, const Entity &_entity)
{
  std::vector<Entity> links;
  if (_ecm.EntityMatches(_entity,
        std::set<ComponentTypeId>{components::Model::typeId}))
  {
    std::stack<Entity> modelStack;
    modelStack.push(_entity);

    std::vector<Entity> childLinks, childModels;
    while (!modelStack.empty())
    {
      Entity model = modelStack.top();
      modelStack.pop();

      childLinks = _ecm.EntitiesByComponents(components::ParentEntity(model),
                                             components::Link());
      links.insert(links.end(),
                   childLinks.begin(),
                   childLinks.end());

      childModels =
          _ecm.EntitiesByComponents(components::ParentEntity(model),
                                    components::Model());
      for (const auto &childModel : childModels)
      {
          modelStack.push(childModel);
      }
    }
  }
  else if (_ecm.EntityMatches(_entity,
              std::set<ComponentTypeId>{components::Link::typeId}))
  {
    links.push_back(_entity);
  }
  return links;
}

/////////////////////////////////////////////////
VisualizationCapabilities::VisualizationCapabilities()
  : GuiSystem(),
  dataPtr(std::make_unique<VisualizationCapabilitiesPrivate>())
{
}

/////////////////////////////////////////////////
VisualizationCapabilities::~VisualizationCapabilities() = default;

//////////////////////////////////////////////////
void VisualizationCapabilities::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
  {
    _ecm.EachNew<components::World, components::Scene>(
      [&](const Entity & _entity,
        const components::World *,
        const components::Scene *)->bool
      {
        this->dataPtr->worldId = _entity;
        return true;
      });

    _ecm.Each<components::Link, components::Name, components::Pose,
          components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Name *_name,
          const components::Pose *,
          const components::ParentEntity *_parent)->bool
      {
        this->dataPtr->modelToLinkEntities[_parent->Data()].push_back(_entity);
        // used for joints
        this->dataPtr->matchLinksWithEntities[_parent->Data()][_name->Data()] =
            _entity;
        return true;
      });

    // inertials
    _ecm.Each<components::Inertial, components::Pose>(
      [&](const Entity &_entity,
          const components::Inertial *_inrElement,
          const components::Pose *) -> bool
      {
        this->dataPtr->entityInertials[_entity] = _inrElement->Data();
        return true;
      });

    // joints
    _ecm.Each<components::Joint, components::Name, components::JointType,
                components::Pose, components::ParentEntity,
                components::ParentLinkName, components::ChildLinkName>(
        [&](const Entity &_entity,
            const components::Joint * /* _joint */,
            const components::Name *_name,
            const components::JointType *_jointType,
            const components::Pose *_pose,
            const components::ParentEntity *_parentModel,
            const components::ParentLinkName *_parentLinkName,
            const components::ChildLinkName *_childLinkName) -> bool
        {
          sdf::Joint joint;
          joint.SetName(_name->Data());
          joint.SetType(_jointType->Data());
          joint.SetRawPose(_pose->Data());

          joint.SetParentName(_parentLinkName->Data());
          joint.SetChildName(_childLinkName->Data());

          auto jointAxis = _ecm.Component<components::JointAxis>(_entity);
          auto jointAxis2 = _ecm.Component<components::JointAxis2>(_entity);

          if (jointAxis)
          {
            joint.SetAxis(0, jointAxis->Data());
          }
          if (jointAxis2)
          {
            joint.SetAxis(1, jointAxis2->Data());
          }

          this->dataPtr->entityJoints[_entity] = joint;
          this->dataPtr->modelToJointEntities[_parentModel->Data()]
            .push_back(_entity);
          return true;
        });

    // visuals
    _ecm.Each<components::Visual, components::Name, components::Pose,
              components::Geometry,
              components::CastShadows,
              components::Transparency,
              components::VisibilityFlags,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::Name *,
            const components::Pose *,
            const components::Geometry *,
            const components::CastShadows *,
            const components::Transparency *,
            const components::VisibilityFlags *,
            const components::ParentEntity *_parent)->bool
        {
          this->dataPtr->linkToVisualEntities[_parent->Data()]
            .push_back(_entity);
          return true;
        });

    _ecm.Each<components::Model, components::Name, components::Pose,
              components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *,
          const components::Pose *,
          const components::ParentEntity *_parent)->bool
      {
        this->dataPtr->modelToModelEntities[_parent->Data()].push_back(_entity);
        return true;
      });

    // collisions
    _ecm.Each<components::Collision, components::Name, components::Pose,
              components::Geometry, components::CollisionElement,
              components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Collision *,
          const components::Name *,
          const components::Pose *,
          const components::Geometry *,
          const components::CollisionElement *_collElement,
          const components::ParentEntity *_parent) -> bool
      {
        this->dataPtr->entityCollisions[_entity] = _collElement->Data();
        this->dataPtr->linkToCollisionEntities[_parent->Data()]
          .push_back(_entity);
        return true;
      });

    // entities with pose
    _ecm.Each<components::Pose, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Pose *,
          const components::ParentEntity *_parent) -> bool
      {
        this->dataPtr->entitiesWithPose[_entity] = _parent->Data();
        return true;
      });

    this->dataPtr->initialized = true;
  }
  else
  {
    _ecm.EachNew<components::World, components::Scene>(
      [&](const Entity & _entity,
        const components::World *,
        const components::Scene *)->bool
      {
        this->dataPtr->worldId = _entity;
        return true;
      });

    _ecm.EachNew<components::Link, components::Name, components::Pose,
          components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Link *,
          const components::Name *_name,
          const components::Pose *,
          const components::ParentEntity *_parent)->bool
      {
        this->dataPtr->modelToLinkEntities[_parent->Data()].push_back(_entity);
        // used for joints
        this->dataPtr->matchLinksWithEntities[_parent->Data()][_name->Data()] =
            _entity;
        return true;
      });

    // inertials
    _ecm.EachNew<components::Inertial, components::Pose>(
      [&](const Entity &_entity,
          const components::Inertial *_inrElement,
          const components::Pose *) -> bool
      {
        this->dataPtr->entityInertials[_entity] = _inrElement->Data();
        return true;
      });

    // visuals
    _ecm.EachNew<components::Visual, components::Name, components::Pose,
              components::Geometry,
              components::CastShadows,
              components::Transparency,
              components::VisibilityFlags,
              components::ParentEntity>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::Name *,
            const components::Pose *,
            const components::Geometry *,
            const components::CastShadows *,
            const components::Transparency *,
            const components::VisibilityFlags *,
            const components::ParentEntity *_parent)->bool
        {
          this->dataPtr->linkToVisualEntities[_parent->Data()]
            .push_back(_entity);
          return true;
        });

    // joints
    _ecm.EachNew<components::Joint, components::Name, components::JointType,
                components::Pose, components::ParentEntity,
                components::ParentLinkName, components::ChildLinkName>(
        [&](const Entity &_entity,
            const components::Joint * /* _joint */,
            const components::Name *_name,
            const components::JointType *_jointType,
            const components::Pose *_pose,
            const components::ParentEntity *_parentModel,
            const components::ParentLinkName *_parentLinkName,
            const components::ChildLinkName *_childLinkName) -> bool
        {
          sdf::Joint joint;
          joint.SetName(_name->Data());
          joint.SetType(_jointType->Data());
          joint.SetRawPose(_pose->Data());

          joint.SetParentName(_parentLinkName->Data());
          joint.SetChildName(_childLinkName->Data());

          auto jointAxis = _ecm.Component<components::JointAxis>(_entity);
          auto jointAxis2 = _ecm.Component<components::JointAxis2>(_entity);

          if (jointAxis)
          {
            joint.SetAxis(0, jointAxis->Data());
          }
          if (jointAxis2)
          {
            joint.SetAxis(1, jointAxis2->Data());
          }

          this->dataPtr->entityJoints[_entity] = joint;
          this->dataPtr->modelToJointEntities[_parentModel->Data()]
            .push_back(_entity);
          return true;
        });

    _ecm.EachNew<components::Model, components::Name, components::Pose,
              components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Model *,
          const components::Name *,
          const components::Pose *,
          const components::ParentEntity *_parent)->bool
      {
        this->dataPtr->modelToModelEntities[_parent->Data()].push_back(_entity);
        return true;
      });

    // collisions
    _ecm.EachNew<components::Collision, components::Name, components::Pose,
              components::Geometry, components::CollisionElement,
              components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Collision *,
          const components::Name *,
          const components::Pose *,
          const components::Geometry *,
          const components::CollisionElement *_collElement,
          const components::ParentEntity *_parent) -> bool
      {
        this->dataPtr->entityCollisions[_entity] = _collElement->Data();
        this->dataPtr->linkToCollisionEntities[_parent->Data()]
          .push_back(_entity);
        return true;
      });

    // entities with pose
    _ecm.EachNew<components::Pose, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Pose *,
          const components::ParentEntity *_parent) -> bool
      {
        this->dataPtr->entitiesWithPose[_entity] = _parent->Data();
        return true;
      });
  }

  _ecm.EachRemoved<components::Model>(
    [&](const Entity &_entity, const components::Model *)->bool
    {
      this->dataPtr->modelToLinkEntities.erase(_entity);
      this->dataPtr->modelToModelEntities.erase(_entity);
      return true;
    });

  // joints
  _ecm.EachRemoved<components::Joint>(
      [&](const Entity &_entity, const components::Joint *)->bool
      {
        this->dataPtr->entityJoints.erase(_entity);
        this->dataPtr->viewingJoints.erase(_entity);
        return true;
      });

  _ecm.EachRemoved<components::Link>(
    [&](const Entity &_entity, const components::Link *)->bool
    {
      this->dataPtr->linkToVisualEntities.erase(_entity);
      return true;
    });

  // entities with pose
  _ecm.EachRemoved<components::Pose>(
    [&](const Entity &_entity,
        const components::Pose *) -> bool
    {
      this->dataPtr->entitiesWithPose.erase(_entity);
      return true;
    });

  this->dataPtr->PopulateViewModeVisualLinks(_ecm);
  this->dataPtr->FindInertialLinks(_ecm);
  this->dataPtr->FindJointModels(_ecm);
  this->dataPtr->FindCollisionLinks(_ecm);
}

/////////////////////////////////////////////////
void VisualizationCapabilities::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Visualization capabilities";

  static bool done{false};
  if (done)
  {
    std::string msg{
        "Only one Visualization capabilities plugin is supported at a time."};
    gzerr << msg << std::endl;
    QQmlProperty::write(this->PluginItem(), "message",
        QString::fromStdString(msg));
    return;
  }
  done = true;

  // view as transparent service
  this->dataPtr->viewTransparentService = "/gui/view/transparent";
  this->dataPtr->node.Advertise(this->dataPtr->viewTransparentService,
    &VisualizationCapabilitiesPrivate::OnViewTransparent, this->dataPtr.get());
  gzmsg << "View as transparent service on ["
         << this->dataPtr->viewTransparentService << "]" << std::endl;

  // view wireframes service
  this->dataPtr->viewWireframesService = "/gui/view/wireframes";
  this->dataPtr->node.Advertise(this->dataPtr->viewWireframesService,
   &VisualizationCapabilitiesPrivate::OnViewWireframes, this->dataPtr.get());
  gzmsg << "View as wireframes service on ["
        << this->dataPtr->viewWireframesService << "]" << std::endl;

  // view center of mass service
  this->dataPtr->viewCOMService = "/gui/view/com";
  this->dataPtr->node.Advertise(this->dataPtr->viewCOMService,
      &VisualizationCapabilitiesPrivate::OnViewCOM, this->dataPtr.get());
  gzmsg << "View center of mass service on ["
         << this->dataPtr->viewCOMService << "]" << std::endl;

  // view inertia service
  this->dataPtr->viewInertiaService = "/gui/view/inertia";
  this->dataPtr->node.Advertise(this->dataPtr->viewInertiaService,
      &VisualizationCapabilitiesPrivate::OnViewInertia, this->dataPtr.get());
  gzmsg << "View inertia service on ["
         << this->dataPtr->viewInertiaService << "]" << std::endl;

   // view collisions service
   this->dataPtr->viewCollisionsService = "/gui/view/collisions";
   this->dataPtr->node.Advertise(this->dataPtr->viewCollisionsService,
       &VisualizationCapabilitiesPrivate::OnViewCollisions,
       this->dataPtr.get());
   gzmsg << "View collisions service on ["
          << this->dataPtr->viewCollisionsService << "]" << std::endl;

  // view joints service
  this->dataPtr->viewJointsService = "/gui/view/joints";
  this->dataPtr->node.Advertise(this->dataPtr->viewJointsService,
      &VisualizationCapabilitiesPrivate::OnViewJoints,
      this->dataPtr.get());
  gzmsg << "View joints service on ["
         << this->dataPtr->viewJointsService << "]" << std::endl;

  // view frames service
  this->dataPtr->viewFramesService = "/gui/view/frames";
  this->dataPtr->node.Advertise(this->dataPtr->viewFramesService,
      &VisualizationCapabilitiesPrivate::OnViewFrames, this->dataPtr.get());
  gzmsg << "View frames service on ["
         << this->dataPtr->viewFramesService << "]" << std::endl;

  gz::gui::App()->findChild
    <gz::gui::MainWindow *>()->installEventFilter(this);
}

////////////////////////////////////////////////
bool VisualizationCapabilities::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  return QObject::eventFilter(_obj, _event);
}


// Register this plugin
GZ_ADD_PLUGIN(gz::sim::VisualizationCapabilities,
                    gz::gui::Plugin)
