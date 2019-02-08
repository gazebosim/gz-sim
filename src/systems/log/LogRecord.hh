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
#ifndef IGNITION_GAZEBO_SYSTEMS_LOGRECORD_HH_
#define IGNITION_GAZEBO_SYSTEMS_LOGRECORD_HH_

#include <memory>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/System.hh>

//#include "ignition/gazebo/Model.hh"

// Use ign-transport directly
#include <ignition/transport/log/Recorder.hh>


namespace ignition
{
namespace gazebo
{
namespace systems
{
  // Inline bracket to help doxygen filtering.
  inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \class LogRecord LogRecord.hh ignition/gazebo/systems/log/LogRecord.hh
  /// \brief Log state recorder
  class IGNITION_GAZEBO_VISIBLE LogRecord:
    public System,
    public ISystemConfigure,
    public ISystemUpdate
  {
    /// \brief Constructor
    public: explicit LogRecord();

    /// \brief Destructor
    public: ~LogRecord() override;

    /// Documentation inherited
    public: void Configure(const Entity &_id,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) final;

    //public: void PreUpdate(const UpdateInfo &_info,
    //                       EntityComponentManager &_ecm);

    /// Documentation inherited
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) final;

    /// \brief A private entity component manager to store a copy of all
    /// entities and components (just for fun).
    //private: EntityComponentManager entityCompMgr;

    // If use ign-transport Log, must end in .tlog
    /// \brief Name of log file to record
    public: std::string logPath = "file.tlog";


    // Use ign-transport directly
    /// \brief log file or nullptr if not recording
    private: ignition::transport::log::Recorder recorder;

    // Just for testing
    /// \brief Model interface
    //private: Model model{kNullEntity};
  };
  }
}
}
}
#endif
