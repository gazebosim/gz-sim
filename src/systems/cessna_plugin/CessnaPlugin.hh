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
 
#ifndef IGNITION_GAZEBO_SYSTEMS_CESSNA_PLUGIN_HH_
#define IGNITION_GAZEBO_SYSTEMS_CESSNA_PLUGIN_HH_

#include <memory>
#include "ignition/gazebo/System.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{  
   // Forward declaration
   class CessnaPluginPrivate;
    
   /// \brief Allow moving the control surfaces of a Cessna C-172 plane. This
   /// plugin might be used with other models that have similar control surfaces.
   
   /// Parameters
   /// The plugin requires the following parameters:
   /// <propeller>         Name of the joint controlling the propeller spin.
   /// <propeller_max_rpm> Maximum angular speed in rpm.
   /// <left_aileron>      Name of the joint controlling the left aileron.
   /// <left_flap>         Name of the joint controlling the left flap.
   /// <right_aileron>     Name of the joint controlling the right aileron.
   /// <right_flap>        Name of the joint controlling the right flap.
   /// <elevators>         Name of the joint controlling the rear elevators.
   /// <rudder>            Name of the joint controlling the rudder.
   ///
   /// The following parameters are optional:
   /// <propeller_p_gain> P gain for the PID that controls the propeller's speed.
   /// <propeller_i_gain> I gain for the PID that controls the propeller's speed.
   /// <propeller_d_gain> D gain for the PID that controls the propeller's speed.
   /// <surfaces_p_gain> P gain for the PID that controls the position of the
   ///                   control surfaces.
   /// <surfaces_i_gain> I gain for the PID that controls the position of the
   ///                   control surfaces.
   /// <surfaces_d_gain> D gain for the PID that controls the position of the
   ///                   control surfaces.
   ///	

   /// <----Include topic-related info here----------->

   class CessnaPlugin : 
        public System,
        public ISystemConfigure,
        public ISystemPreUpdate

   {   
      /// \brief Constructor.
      public: CessnaPlugin();

      /// \brief Destructor.
      public: ~CessnaPlugin() override;

      // Documentation inherited
      public: void Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &_eventMgr) override;

      // Documentation inherited
      public: void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                  ignition::gazebo::EntityComponentManager &_ecm) override;

      /// \brief Private data pointer
      private: std::unique_ptr<CessnaPluginPrivate> dataPtr;
   };
   }
}
}
}
#endif 