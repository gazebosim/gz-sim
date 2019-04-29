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
#ifndef IGNITION_GAZEBO_RENDERING_HH_
#define IGNITION_GAZEBO_RENDERING_HH_

#include <memory>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  // forward declaration
  class RenderUtilPrivate;

  /// \class RenderUtil RenderUtil.hh ignition/gazebo/gui/plugins/RenderUtil.hh
  class IGNITION_GAZEBO_VISIBLE RenderUtil
  {
    /// \brief Constructor
    public: explicit RenderUtil();

    /// \brief Destructor
    public: ~RenderUtil();

    /// \brief Initialize the renderer. Must be called in the rendering thread.
    public: void Init();

    /// \brief Main update function. Must be called in the rendering thread.
    public: void Update();

    /// \brief Get a pointer to the scene
    /// \return Pointer to the scene
    public: rendering::ScenePtr Scene() const;

    /// \brief Helper PostUpdate function for updating the scene
    public: void UpdateFromECM(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm);

    /// \brief Set the rendering engine to use
    /// \param[in] _engineName Name of the rendering engine.
    public: void SetEngineName(const std::string &_engineName);
    public: std::string EngineName() const;

    /// \brief Set the scene to use
    /// \param[in] _sceneName Name of the engine.
    public: void SetSceneName(const std::string &_sceneName);
    public: std::string SceneName() const;

    /// \brief Set background color of render window
    /// \param[in] _color Color of render window background
    public: void SetBackgroundColor(const math::Color &_color);

    /// \brief Set ambient light of render window
    /// \param[in] _ambient Color of ambient light
    public: void SetAmbientLight(const math::Color &_ambient);

    /// \brief Set the initial pose the render window camera
    /// \param[in] _pose Initical camera pose
    public: void SetCameraPose(const math::Pose3d &_pose);
    public: math::Pose3d CameraPose() const;

    /// \brief Set whether to use the current GL context
    /// \param[in] _enable True to use the current GL context
    public: void SetUseCurrentGLContext(bool _enable);

    /// \brief Get a pointer to the scene
    /// \return Pointer to the scene
    public: void SetEnableSensors(bool _enable, const std::function<
        unsigned int(uint64_t, sdf::ElementPtr, std::string)>
        &_createSensorCb = {});

    /// \brief Private data pointer.
    private: std::unique_ptr<RenderUtilPrivate> dataPtr;
  };
}
}
}
#endif
