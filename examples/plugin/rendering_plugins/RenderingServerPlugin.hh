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
#ifndef RENDERING_SERVER_PLUGIN_HH_
#define RENDERING_SERVER_PLUGIN_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/rendering/Scene.hh>

/// \brief Server-side system that uses Ignition Rendering APIs.
/// It changes the ambient color every 2 simulation seconds.
class RenderingServerPlugin:
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
  /// \brief Called once at startup
  /// \param[in] _entity Entity that the plugin is attached to, not used.
  /// \param[in] _sdf Element with custom configuration, not used.
  /// \param[in] _ecm Entity component manager
  /// \param[in] _eventMgr Event manager
  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;

  /// \brief Called just before each simulation update.
  /// \param[in] _info Contains information like sim time.
  /// \param[in] _ecm Entity component manager
  public: void PreUpdate(
              const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;

  /// \brief All rendering operations must happen within this call
  private: void PerformRenderingOperations();

  /// \brief Encapsulates the logic to find the rendering scene through the
  /// render engine singleton.
  private: void FindScene();

  /// \brief Connection to pre-render event callback
  private: ignition::common::ConnectionPtr connection{nullptr};

  /// \brief Pointer to rendering scene
  private: ignition::rendering::ScenePtr scene{nullptr};

  /// \brief Current simulation time.
  private: std::chrono::steady_clock::duration simTime{0};

  /// \brief Time when light was last updated
  private: std::chrono::steady_clock::duration lastUpdate{0};
};

#endif
