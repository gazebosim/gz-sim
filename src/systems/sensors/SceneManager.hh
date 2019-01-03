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
namespace systems
{
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  //
  // Forward declaration
  class SceneManagerPrivate;

  /// \brief Scene manager class for loading and managing objects in the scene
  class IGNITION_GAZEBO_VISIBLE SceneManager
  {
    /// \brief Constructor
    public: SceneManager();

    /// \brief Destructor
    public: ~SceneManager();

    /// \brief Set the scene to manager
    public: void SetScene(rendering::ScenePtr _scene);

    /// \brief Create a model
    /// \param[in] _id Unique model id
    /// \param[in] _model Model sdf dom
    /// \param[in] _parentId Parent id
    /// \return Model visual created from the sdf dom
    public: rendering::VisualPtr CreateModel(int _id, const sdf::Model &_model,
        int _parentId = -1);

    /// \brief Create a link
    /// \param[in] _id Unique link id
    /// \param[in] _link Link sdf dom
    /// \param[in] _parentId Parent id
    /// \return Link visual created from the sdf dom
    public: rendering::VisualPtr CreateLink(int _id, const sdf::Link &_link,
        int _parentId = -1);

    /// \brief Create a visual
    /// \param[in] _id Unique visual id
    /// \param[in] _visual Visual sdf dom
    /// \param[in] _parentId Parent id
    /// \return Visual object created from the sdf dom
    public: rendering::VisualPtr CreateVisual(int _id,
        const sdf::Visual &_visual, int _parentId = -1);

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
    public: rendering::LightPtr CreateLight(int _id, const sdf::Light &_light,
        int _parentId);

    /// \brief Add an existing sensor to the scene
    /// \param[in] _name Name of sensor
    /// \param[in] _parentId Parent Id
    /// \return True if sensor is successfully added
    public: bool AddSensor(int _id, const std::string &_name,
        int _parentId = -1);

    /// \brief Check if entity exists
    /// \param[in] _id Unique entity id
    /// \return true if exists, false otherwise
    public: bool HasEntity(int _id) const;

    /// \brief Get an entity by id
    /// \param[in] _id Entity's unique id
    /// \return Pointer to requested entity
    public: rendering::NodePtr EntityById(int _id) const;

    /// \internal
    /// \brief Pointer to private data class
    private: std::unique_ptr<SceneManagerPrivate> dataPtr;
  };
  }
}
}
}

#endif
