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

#ifndef IGNITION_GAZEBO_SYSTEMS_SCENEMANAGER_HH_
#define IGNITION_GAZEBO_SYSTEMS_SCENEMANAGER_HH_

#include <map>
#include <memory>
#include <string>

#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Light.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>
#include <sdf/Visual.hh>

#include <ignition/common/MeshManager.hh>
#include <ignition/rendering.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
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

    /// \brief Set the world's ID.
    /// \param[in] _id World ID.
    public: void SetWorldId(uint64_t _id);

    /// \brief Create a model
    /// \param[in] _id Unique model id
    /// \param[in] _model Model sdf dom
    /// \param[in] _parentId Parent id
    /// \return Model visual created from the sdf dom
    public: rendering::VisualPtr CreateModel(uint64_t _id,
        const sdf::Model &_model, uint64_t _parentId = 0);

    /// \brief Create a link
    /// \param[in] _id Unique link id
    /// \param[in] _link Link sdf dom
    /// \param[in] _parentId Parent id
    /// \return Link visual created from the sdf dom
    public: rendering::VisualPtr CreateLink(uint64_t _id,
        const sdf::Link &_link, uint64_t _parentId = 0);

    /// \brief Create a visual
    /// \param[in] _id Unique visual id
    /// \param[in] _visual Visual sdf dom
    /// \param[in] _parentId Parent id
    /// \return Visual object created from the sdf dom
    public: rendering::VisualPtr CreateVisual(uint64_t _id,
        const sdf::Visual &_visual, uint64_t _parentId = 0);

    /// \brief Load a geometry
    /// \param[in] __geom Geometry sdf dom
    /// \param[out] _scale Geometry scale that will be set based on sdf
    /// \param[out] _localPose Additional local pose to be applied after the
    /// visual's pose
    /// \return Geometry object loaded from the sdf dom
    public: rendering::GeometryPtr LoadGeometry(const sdf::Geometry &_geom,
        math::Vector3d &_scale, math::Pose3d &_localPose);

    /// \brief Load a material
    /// \param[in] _material Material sdf dom
    /// \return Material object loaded from the sdf dom
    public: rendering::MaterialPtr LoadMaterial(
        const sdf::Material &_material);

    /// \brief Create a light
    /// \param[in] _id Unique light id
    /// \param[in] _light Light sdf dom
    /// \param[in] _parentId Parent id
    /// \return Light object created from the sdf dom
    public: rendering::LightPtr CreateLight(uint64_t _id,
        const sdf::Light &_light, uint64_t _parentId);

    /// \brief Ignition sensors is the one responsible for adding sensors
    /// to the scene. Here we just keep track of it and make sure it has
    /// the correct parent.
    /// \param[in] _gazeboId Entity in Gazebo
    /// \param[in] _renderingId ID of sensor node in Ignition Rendering.
    /// \param[in] _parentId Parent Id on Gazebo.
    /// \return True if sensor is successfully handled
    public: bool AddSensor(uint64_t _gazeboId, uint64_t _renderingId,
        uint64_t _parentGazeboId = 0);

    /// \brief Check if entity exists
    /// \param[in] _id Unique entity id
    /// \return true if exists, false otherwise
    public: bool HasEntity(uint64_t _id) const;

    /// \brief Get a rendering node given an id
    /// \param[in] _id Entity's unique id
    /// \return Pointer to requested entity's node
    public: rendering::NodePtr NodeById(uint64_t _id) const;

    /// \brief Remove an entity by id
    /// \param[in] _id Entity's unique id
    public: void RemoveEntity(uint64_t _id);

    /// \internal
    /// \brief Pointer to private data class
    private: std::unique_ptr<SceneManagerPrivate> dataPtr;
  };
  }
}
}
}

#endif
