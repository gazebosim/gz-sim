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

#ifndef GZ_SIM_SCENEMANAGER_HH_
#define GZ_SIM_SCENEMANAGER_HH_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sdf/Geometry.hh>
#include <sdf/Actor.hh>
#include <sdf/Joint.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Model.hh>
#include <sdf/Visual.hh>

#include <gz/common/KeyFrame.hh>
#include <gz/common/Animation.hh>
#include <gz/common/graphics/Types.hh>

#include <gz/math/Inertial.hh>
#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

#include <gz/msgs/particle_emitter.pb.h>

#include <gz/rendering/RenderTypes.hh>

#include <gz/sim/config.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/rendering/Export.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
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
  class GZ_SIM_RENDERING_VISIBLE SceneManager
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

    /// \brief Set the spherical coordinates from world.
    /// \param[in] _sphericalCoordinates SphericalCoordinates
    /// from the world.
    public: void SetSphericalCoordinates(
        const math::SphericalCoordinates &_sphericalCoordinates);

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

    /// \brief Filter a node and its children according to specific criteria.
    /// \param[in] _node The name of the node where filtering should start.
    /// \param[in] _filter Callback function that defines how _node and its
    /// children should be filtered. The function parameter is a node. The
    /// callback returns true if the node should be filtered; false otherwise.
    /// \return A list of filtered nodes in top level order. This list can
    /// contain _node itself, or child nodes of _node. An empty list means no
    /// nodes were filtered.
    public: std::vector<rendering::NodePtr> Filter(const std::string &_node,
                std::function<bool(
                  const rendering::NodePtr _nodeToFilter)> _filter) const;

    /// \brief Copy a visual that currently exists in the scene
    /// \param[in] _id Unique visual id of the copied visual
    /// \param[in] _visual Name of the visual to copy
    /// \param[in] _parentId Parent id of the copied visual
    /// \return A pair with the first element being the copied visual object,
    /// and the second element being a list of the entity IDs for the copied
    /// visual's children, in level order. If copying the visual failed, the
    /// first element will be nullptr. If the copied visual has no children, the
    /// second element will be empty.
    public: std::pair<rendering::VisualPtr, std::vector<Entity>> CopyVisual(
                Entity _id, const std::string &_visual, Entity _parentId = 0);

    /// \brief Create a visual
    /// \param[in] _id Unique visual id
    /// \param[in] _visual Visual sdf dom
    /// \param[in] _parentId Parent id
    /// \return Visual object created from the sdf dom
    public: rendering::VisualPtr CreateVisual(Entity _id,
        const sdf::Visual &_visual, Entity _parentId = 0);

    /// \brief Create a center of mass visual
    /// \param[in] _id Unique visual id
    /// \param[in] _inertial Inertial component of the link
    /// \param[in] _parentId Parent id
    /// \return Visual (center of mass) object created from the inertial
    public: rendering::VisualPtr CreateCOMVisual(Entity _id,
        const math::Inertiald &_inertial, Entity _parentId = 0);

    /// \brief Create an inertia visual
    /// \param[in] _id Unique visual id
    /// \param[in] _inertial Inertial component of the link
    /// \param[in] _parentId Parent id
    /// \return Visual (inertia) object created from the inertial
    public: rendering::VisualPtr CreateInertiaVisual(Entity _id,
        const math::Inertiald &_inertial, Entity _parentId = 0);

    /// \brief Create a joint visual
    /// \param[in] _id Unique visual id
    /// \param[in] _joint Joint sdf dom
    /// \param[in] _childId Joint child id
    /// \param[in] _parentId Joint parent id
    /// \return Visual (joint) object created from the sdf dom
    public: rendering::VisualPtr CreateJointVisual(Entity _id,
        const sdf::Joint &_joint, Entity _childId = 0,
        Entity _parentId = 0);

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

    /// \brief Load Actor animations
    /// \param[in] _actor Actor
    /// \return Animation name to ID map
    public: std::unordered_map<std::string, unsigned int>
        LoadAnimations(const sdf::Actor &_actor);

    /// \brief Sequences Trajectories
    /// \param[in] _trajectories Actor trajectories
    /// \param[in] _time Actor trajectory delay start time (miliseconds)
    public: void SequenceTrajectories(
        std::vector<common::TrajectoryInfo>& _trajectories,
        std::chrono::steady_clock::time_point _time);

    /// \brief Create an actor
    /// \param[in] _id Unique actor id
    /// \param[in] _actor Actor sdf dom
    /// \param[in] _name Actor's name
    /// \param[in] _parentId Parent id
    /// \return Actor object created from the sdf dom
    public: rendering::VisualPtr CreateActor(Entity _id,
        const sdf::Actor &_actor, const std::string &_name,
        Entity _parentId = 0);

    /// \brief Create a light
    /// \param[in] _id Unique light id
    /// \param[in] _light Light sdf dom
    /// \param[in] _name Light's name
    /// \param[in] _parentId Parent id
    /// \return Light object created from the sdf dom
    public: rendering::LightPtr CreateLight(Entity _id,
        const sdf::Light &_light, const std::string &_name, Entity _parentId);

    /// \brief Create a light
    /// \param[in] _id Unique light id
    /// \param[in] _light Light sdf dom
    /// \param[in] _name Light's name
    /// \param[in] _parentId Parent id
    /// \return Light object created from the sdf dom
    public: rendering::VisualPtr CreateLightVisual(Entity _id,
        const sdf::Light &_light, const std::string &_name, Entity _parentId);

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

    /// \brief Create a projector
    /// \param[in] _id Unique projector id
    /// \param[in] _projector Projector sdf dom
    /// \param[in] _parentId Parent id
    /// \return Projector object created from the sdf dom
    public: rendering::ProjectorPtr CreateProjector(
        Entity _id, const sdf::Projector &_projector, Entity _parentId);

    /// \brief Gazebo Sensors is the one responsible for adding sensors
    /// to the scene. Here we just keep track of it and make sure it has
    /// the correct parent.
    /// \param[in] _simId Entity in Sim
    /// \param[in] _sensorName Name of sensor node in Gazebo Rendering.
    /// \param[in] _parentGazeboId Parent Id on Gazebo.
    /// \return True if sensor is successfully handled
    public: bool AddSensor(Entity _simId, const std::string &_sensorName,
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
    /// \return Top level visual containing this visual
    public: rendering::VisualPtr TopLevelVisual(
        const rendering::VisualPtr &_visual) const;

    /// \brief Get the top level node for the given node, which
    /// is the ancestor which is a direct child to the root visual.
    /// Usually, this will be a model or a light.
    /// \param[in] _node Child node
    /// \return Top level node containing this node
    public: rendering::NodePtr TopLevelNode(
        const rendering::NodePtr &_node) const;

    /// \brief Updates the node to increase its transparency or reset
    /// back to its original transparency value, an opaque call requires
    /// a previous transparent call, otherwise, no action will be taken
    /// Usually, this will be a link visual
    /// \param[in] _node The node to update.
    /// \param[in] _makeTransparent true if updating to increase transparency,
    /// false to set back to original transparency values (make more opaque)
    public: void UpdateTransparency(const rendering::NodePtr &_node,
        bool _makeTransparent);

    /// \brief Updates the world pose of joint parent visual
    /// according to its child.
    /// \param[in] _jointId Joint visual id.
    public: void UpdateJointParentPose(Entity _jointId);

    /// \brief Create a unique entity ID
    /// \return A unique entity ID. kNullEntity is returned if no unique entity
    /// IDs are available
    public: Entity UniqueId() const;

    /// \brief Clear the scene manager
    /// Clears internal resources stored in the scene manager.
    /// Note: this does not actually destroy the objects.
    public: void Clear();

    /// \internal
    /// \brief Pointer to private data class
    private: std::unique_ptr<SceneManagerPrivate> dataPtr;
  };
}
}
}

#endif
