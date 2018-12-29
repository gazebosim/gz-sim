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
  /// \brief Scene manager class for loading and managing objects in the scene
  class IGNITION_GAZEBO_VISIBLE SceneManager
  {
    /// \brief Constructor
    public: SceneManager();

    /// \brief Set the scene to manager
    public: void SetScene(rendering::ScenePtr _scene);

    /// \brief Update the scene based on pose msgs received
    public: void Update();

    /// \brief Callback function for the pose topic
    /// \param[in] _msg Pose vector msg
    // private: void OnPoseVMsg(const msgs::Pose_V &_msg);

    /// \brief Load the model from a model msg
    /// \param[in] _msg Model msg
    /// \return Model visual created from the msg
    public: rendering::VisualPtr LoadModel(int _id, const sdf::Model &_model,
        int _parentId = -1);

    /// \brief Load a link from a link msg
    /// \param[in] _msg Link msg
    /// \return Link visual created from the msg
    public: rendering::VisualPtr LoadLink(int _id, const sdf::Link &_link,
        int _parentId = -1);

    /// \brief Load a visual from a visual msg
    /// \param[in] _msg Visual msg
    /// \return Visual visual created from the msg
    public: rendering::VisualPtr LoadVisual(int _id,
        const sdf::Visual &_visual, int _parentId = -1);

    /// \brief Load a geometry from a geometry msg
    /// \param[in] _msg Geometry msg
    /// \param[out] _scale Geometry scale that will be set based on msg param
    /// \param[out] _localPose Additional local pose to be applied after the
    /// visual's pose
    /// \return Geometry object created from the msg
    public: rendering::GeometryPtr LoadGeometry(const sdf::Geometry &_geom,
        math::Vector3d &_scale, math::Pose3d &_localPose);

    /// \brief Load a material from a material msg
    /// \param[in] _msg Material msg
    /// \return Material object created from the msg
    public: rendering::MaterialPtr LoadMaterial(
        const sdf::Material &_material);

    /// \brief Load a light from a light msg
    /// \param[in] _msg Light msg
    /// \return Light object created from the msg
    public: rendering::LightPtr LoadLight(int _id, const sdf::Light &_light,
        int _parentId);

    /// \brief Check if entity exists
    /// \param[in] _id Unique entity id
    /// \return true if exists, false otherwise
    public: bool HasEntity(int _id) const;

    //// \brief Ign-transport scene service name
    private: std::string service;

    //// \brief Ign-transport pose topic name
    private: std::string poseTopic;

    //// \brief Pointer to the rendering scene
    private: rendering::ScenePtr scene;

    //// \brief Mutex to protect the pose msgs
    private: std::mutex mutex;

    /// \brief Map of entity id to pose
    private: std::map<unsigned int, math::Pose3d> poses;

    /// \brief Map of entity id to initial local poses
    /// This is currently used to handle the normal vector in plane visuals. In
    /// general, this can be used to store any local transforms between the
    /// parent Visual and geometry.
    private: std::map<unsigned int, math::Pose3d> localPoses;

    /// \brief Map of visual id to visual pointers.
    private: std::map<unsigned int, rendering::VisualPtr> visuals;

    /// \brief Map of light id to light pointers.
    private: std::map<unsigned int, rendering::LightPtr> lights;
  };
  }
}
}
}

#endif
