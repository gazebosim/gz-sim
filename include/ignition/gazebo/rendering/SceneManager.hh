/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef IGNITION_GAZEBO_SCENEMANAGER_HH_
#define IGNITION_GAZEBO_SCENEMANAGER_HH_

#include <map>
#include <memory>
#include <string>

#include <sdf/Geometry.hh>
#include <sdf/Actor.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>

#include <ignition/common/KeyFrame.hh>

#include <ignition/rendering/RenderTypes.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declaration
  class SceneManagerPrivate;

  /// \brief Scene manager class for loading and managing objects in the scene
  class IGNITION_GAZEBO_VISIBLE SceneManager
  {
    /// \brief Constructor
    public: SceneManager();

    /// \brief Destructor
    public: ~SceneManager();

    /// \brief Set the scene to manage
    /// \param[in] _scene Scene pointer.
    public: void SetScene(rendering::ScenePtr _scene);

    /// \brief Get the scene
    /// \return Pointer to scene
    public: rendering::ScenePtr Scene() const;

    /// \brief Set the world's ID.
    /// \param[in] _id World ID.
    public: void SetWorldId(Entity _id);

    /// \brief Create a model
    /// \param[in] _id Unique model id
    /// \param[in] _model Model sdf dom
    /// \param[in] _parentId Parent id
    /// \return Model visual created from the sdf dom
    public: rendering::VisualPtr CreateModel(Entity _id,
        const sdf::Model &_model, Entity _parentId = 0);

    /// \brief Create a link
    /// \param[in] _id Unique link id
    /// \param[in] _link Link sdf dom
    /// \param[in] _parentId Parent id
    /// \return Link visual created from the sdf dom
    public: rendering::VisualPtr CreateLink(Entity _id,
        const sdf::Link &_link, Entity _parentId = 0);

    /// \brief Create a visual
    /// \param[in] _id Unique visual id
    /// \param[in] _visual Visual sdf dom
    /// \param[in] _parentId Parent id
    /// \return Visual object created from the sdf dom
    public: rendering::VisualPtr CreateVisual(Entity _id,
        const sdf::Visual &_visual, Entity _parentId = 0);

    /// \brief Create an actor
    /// \param[in] _id Unique actor id
    /// \param[in] _visual Actor sdf dom
    /// \param[in] _parentId Parent id
    /// \return Actor object created from the sdf dom
    public: rendering::VisualPtr CreateActor(Entity _id,
        const sdf::Actor &_actor, Entity _parentId = 0);

    /// \brief Create a light
    /// \param[in] _id Unique light id
    /// \param[in] _light Light sdf dom
    /// \param[in] _parentId Parent id
    /// \return Light object created from the sdf dom
    public: rendering::LightPtr CreateLight(Entity _id,
        const sdf::Light &_light, Entity _parentId);

    /// \brief Ignition sensors is the one responsible for adding sensors
    /// to the scene. Here we just keep track of it and make sure it has
    /// the correct parent.
    /// \param[in] _gazeboId Entity in Gazebo
    /// \param[in] _sensorName Name of sensor node in Ignition Rendering.
    /// \param[in] _parentId Parent Id on Gazebo.
    /// \return True if sensor is successfully handled
    public: bool AddSensor(Entity _gazeboId, const std::string &_sensorName,
        Entity _parentGazeboId = 0);

    /// \brief Check if entity exists
    /// \param[in] _id Unique entity id
    /// \return true if exists, false otherwise
    public: bool HasEntity(Entity _id) const;

    /// \brief Get a rendering node given an entity id
    /// \param[in] _id Entity's unique id
    /// \return Pointer to requested entity's node
    public: rendering::NodePtr NodeById(Entity _id) const;

    /// \brief Get a rendering mesh given an id
    /// \param[in] _id Actor entity's unique id
    /// \return Pointer to requested entity's mesh
    public: rendering::MeshPtr ActorMeshById(Entity _id) const;

    /// \brief Get the animation of actor mesh given an id
    /// \param[in] _id Entity's unique id
    /// \param[in] _time Timepoint for the animation
    /// \return Map from the skeleton node name to transforms
    public: std::map<std::string, math::Matrix4d> ActorMeshAnimationAt(
        Entity _id, std::chrono::steady_clock::duration _time) const;

    /// \brief Remove an entity by id
    /// \param[in] _id Entity's unique id
    public: void RemoveEntity(Entity _id);

    /// \brief Get the entity for a given node.
    /// \param[in] _node Node to get the entity for.
    /// \return The entity for that node, or `kNullEntity` for no entity.
    /// \todo(anyone) Deprecate in favour of
    /// `ignition::rendering::Node::UserData` once that's available.
    public: Entity EntityFromNode(const rendering::NodePtr &_node) const;

    /// \brief Load a geometry
    /// \param[in] _geom Geometry sdf dom
    /// \param[out] _scale Geometry scale that will be set based on sdf
    /// \param[out] _localPose Additional local pose to be applied after the
    /// visual's pose
    /// \return Geometry object loaded from the sdf dom
    private: rendering::GeometryPtr LoadGeometry(const sdf::Geometry &_geom,
        math::Vector3d &_scale, math::Pose3d &_localPose);

    /// \brief Load a material
    /// \param[in] _material Material sdf dom
    /// \return Material object loaded from the sdf dom
    private: rendering::MaterialPtr LoadMaterial(
        const sdf::Material &_material);

    /// \brief Get the top level visual for the given visual, which
    /// is the ancestor which is a direct child to the root visual.
    /// Usually, this will be a model or a light.
    /// \param[in] _visual Child visual
    /// \return Top level visual containining this visual
    /// \TODO(anyone) Make it const ref when merging forward
    public: rendering::VisualPtr TopLevelVisual(
        // NOLINTNEXTLINE
        rendering::VisualPtr _visual) const;

    /// \brief Get the top level node for the given node, which
    /// is the ancestor which is a direct child to the root visual.
    /// Usually, this will be a model or a light.
    /// \param[in] _node Child node
    /// \return Top level node containining this node
    public: rendering::NodePtr TopLevelNode(
        const rendering::NodePtr &_node) const;

    /// \internal
    /// \brief Pointer to private data class
    private: std::unique_ptr<SceneManagerPrivate> dataPtr;
  };
}
}
}

#endif
