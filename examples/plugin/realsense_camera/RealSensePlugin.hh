/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#ifndef IGNITION_GAZEBO_REALSENSE_PLUGIN_HH_
#define IGNITION_GAZEBO_REALSENSE_PLUGIN_HH_

#include <string>
#include <memory>

#include <ignition/gazebo/System.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declarations.
  class RealSensePluginPrivate;

  /// \brief A plugin that simulates Real Sense camera streams
  class RealSensePlugin
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate
  {
  	/// \brief Constructor.
  	public: RealSensePlugin();

  	/// \brief Destructor.
  	public: ~RealSensePlugin();

  	// Documentation Inherited.
  	public: void Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &_eventMgr) override;

  	/// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;

    /// \brief Private data pointer.
    private: std::unique_ptr<RealSensePluginPrivate> dataPtr;
  };
  }
}
}
}

#endif 
