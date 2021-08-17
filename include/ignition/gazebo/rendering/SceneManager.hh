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
#include <ignition/common/Animation.hh>
#include <ignition/common/graphics/Types.hh>

#include <ignition/msgs/particle_emitter.pb.h>

#include <ignition/rendering/RenderTypes.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/gazebo/rendering/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // Forward declaration
  class SceneManagerPrivate;

  /// \brief Data structure for updating skeleton animations
  class AnimationUpdateData
  {
    /// \brief Timepoint in the animation.
    /// Note that animation time is different from sim time. An actor can
    /// have multiple animations. Animation time is associated with
    /// current animation that is being played. This value is also adjusted if
    /// interpolate_x is enabled
    public: std::chrono::steady_clock::duration time;

    /// \brief True if animation is looped
    public: bool loop = false;

    /// \brief True if trajectory animation is on
    public: bool followTrajectory = false;

    /// \brief Trajectory to be followed
    public: common::TrajectoryInfo trajectory;

    /// \brief Name of animation to play. This field is set only if the actor
    /// is not animated by manually using skeleton transforms
    public: std::string animationName;

    /// \brief Transform of the root node in the skeleton. The actor's
    /// skeleton's root node transform needs to be set if trajectory
    /// animation is enabled. This field is set only if the actor
    /// is not animated by manually using skeleton transforms
    public: math::Matrix4d rootTransform;

    /// \brief True if this animation update data is valid. If false, this
    /// update data should be ignored
    public: bool valid = false;
  };

  /// \brief Scene manager class for loading and managing objects in the scene
  class IGNITION_GAZEBO_RENDERING_VISIBLE SceneManager
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

    /// \brief Get the world's ID.
    /// \return World ID
    public: Entity WorldId() const;

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

    /// \brief Create a collision visual
    /// \param[in] _id Unique visual id
    /// \param[in] _collision Collision sdf dom
    /// \param[in] _parentId Parent id
    /// \return Visual (collision) object created from the sdf dom
    public: rendering::VisualPtr CreateCollision(Entity _id,
        const sdf::Collision &_collision, Entity _parentId = 0);

    /// \brief Retrieve visual
    /// \param[in] _id Unique visual (entity) id
    /// \return Pointer to requested visual
    public: rendering::VisualPtr VisualById(Entity _id);

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

    /// \brief Create a particle emitter.
    /// \param[in] _id Unique particle emitter id
    /// \param[in] _emitter Particle emitter data
    /// \param[in] _parentId Parent id
    /// \return Default particle emitter object created
    public: rendering::ParticleEmitterPtr CreateParticleEmitter(
        Entity _id, const msgs::ParticleEmitter &_emitter, Entity _parentId);

    /// \brief Update an existing particle emitter
    /// \brief _id Emitter id to update
    /// \brief _emitter Data to update the particle emitter
    /// \return Particle emitter updated
    public: rendering::ParticleEmitterPtr UpdateParticleEmitter(Entity _id,
        const msgs::ParticleEmitter &_emitter);

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

    /// \brief Get a skeleton given an id
    /// \param[in] _id Actor entity's unique id
    /// \return Pointer to requested entity's skeleton
    public: common::SkeletonPtr ActorSkeletonById(Entity _id) const;

    /// \brief Get the animation of actor mesh given an id
    /// Use this function if you are animating the actor manually by its
    /// skeleton node pose.
    /// \param[in] _id Entity's unique id
    /// \param[in] _time Simulation time
    /// \return Map from the skeleton node name to transforms
    /// \deprecated see ActorSkeletonTransformAt
    public: std::map<std::string, math::Matrix4d> IGN_DEPRECATED(4.0)
        ActorMeshAnimationAt(
        Entity _id, std::chrono::steady_clock::duration _time) const;

    /// \brief Get the skeleton local transforms of actor mesh given an id.
    /// Use this function if you are animating the actor manually by its
    /// skeleton node pose.
    /// \param[in] _id Entity's unique id
    /// \param[in] _time SimulationTime
    /// \return Map from the skeleton node name to transforms
    public: std::map<std::string, math::Matrix4d> ActorSkeletonTransformsAt(
        Entity _id, std::chrono::steady_clock::duration _time) const;

    /// \brief Get the actor animation update data given an id.
    /// Use this function to let the render engine handle the actor animation.
    /// by setting the animation name to be played.
    /// \param[in] _id Entity's unique id
    /// \param[in] _time Simulation time
    /// \return Data needed to update the animation, including the name and
    /// time of animation to play, and trajectory animation info.
    public: AnimationUpdateData ActorAnimationAt(
        Entity _id, std::chrono::steady_clock::duration _time) const;

    /// \brief Remove an entity by id
    /// \param[in] _id Entity's unique id
    public: void RemoveEntity(Entity _id);

    /// \brief Get the entity for a given node.
    /// \param[in] _node Node to get the entity for.
    /// \return The entity for that node, or `kNullEntity` for no entity.
    /// \todo(anyone) Deprecate in favour of
    /// `ignition::rendering::Node::UserData` once that's available.
    public: Entity IGN_DEPRECATED(4)
        EntityFromNode(const rendering::NodePtr &_node) const;

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
    public: rendering::VisualPtr TopLevelVisual(
        const rendering::VisualPtr &_visual) const;

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
