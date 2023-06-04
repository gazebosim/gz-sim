/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifndef GZ_GUI_APPLYFORCETORQUE_HH_
#define GZ_GUI_APPLYFORCETORQUE_HH_

#include <memory>
#include <string>

#include <gz/sim/gui/GuiSystem.hh>
#include <gz/gui/Plugin.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
  class ApplyForceTorquePrivate;

  /// \brief Publish wrench to "/world/apply_link_wrench/wrench" topic.
  ///
  /// ## Configuration
  /// This plugin doesn't accept any custom configuration.
  class ApplyForceTorque
      : public gz::sim::GuiSystem,
        public System,
        public gz::sim::ISystemPreUpdate
  {
    Q_OBJECT

    /// \brief Constructor
    public: ApplyForceTorque();

    /// \brief Destructor
    public: ~ApplyForceTorque() override;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    // Documentation inherited
    public: void Update(const UpdateInfo &_info,
      EntityComponentManager &_ecm) override;

    /// \brief Set X component of force
    /// \param[in] _forceX X component of force
    public: Q_INVOKABLE void UpdateForceX(float _forceX);

    /// \brief Set Y component of force
    /// \param[in] _forceY Y component of force
    public: Q_INVOKABLE void UpdateForceY(float _forceY);

    /// \brief Set Z component of force
    /// \param[in] _forceZ Z component of force
    public: Q_INVOKABLE void UpdateForceZ(float _forceZ);

    /// \brief Set X component of torque
    /// \param[in] _torqueX X component of torque
    public: Q_INVOKABLE void UpdateTorqueX(float _torqueX);

    /// \brief Set Y component of torque
    /// \param[in] _torqueY Y component of torque
    public: Q_INVOKABLE void UpdateTorqueY(float _torqueY);

    /// \brief Set Z component of torque
    /// \param[in] _torqueZ Z component of torque
    public: Q_INVOKABLE void UpdateTorqueZ(float _torqueZ);

    /// \brief Apply the specified force
    public: Q_INVOKABLE void ApplyForce();

    /// \brief Apply the specified torque
    public: Q_INVOKABLE void ApplyTorque();

    /// \brief Apply the specified force and torque
    public: Q_INVOKABLE void ApplyAll();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<ApplyForceTorquePrivate> dataPtr;
  };
}
}

#endif
