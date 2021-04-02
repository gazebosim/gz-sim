/**
Copyright 2014 Konstantinos Chatzilygeroudis
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

/**

Modified by: Adwait Naik on March 31, 2021

**/
///////////////////////////////////////////////////////////////////////////////////


#ifndef IGNITION_GAZEBO_MIMIC_JOINT_PLUGIN_HH
#define IGNITION_GAZEBO_MIMIC_JOINT_PLUGIN_HH

// ROS libraries
#include <ros/ros.h>

// ros_control
#include <control_toolbox/pid.h>

#include "dart/simulation/World.hpp"

#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Physics.hh>

 
namespace igniton 
{
  
namespace gazebo
{

// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
   // Forward declaration
   class MimicJointPrivate;
  
   
   class IGNITION_GAZEBO_VISIBLE MimicJoint:

            public ignition::gazebo::System,
            public ignition::gazebo::ISystemConfigure,
            public ignition::gazebo::ISystemPostUpdate,
            public ignition::gazebo::ISystemPreUpdate 
       {

      	/// \brief Constructor
      	public: MimicJoint();

      	/// \brief Destructor
      	public: ~MimicJoint() override = default;


      	/// \ Implement Configure callback, provided by ISystemConfigure
        /// \ and called once at startup.
        public: void Configure(const Entity & _entity,
					           const std::shared_ptr <
					           const sdf::Element > & _sdf,
					           EntityComponentManager & _ecm,
					           EventManager &_eventMgr) override;

        // Documentation inherited
        public: void PreUpdate(
                    const ignition::gazebo::UpdateInfo &_info,
                    ignition::gazebo::EntityComponentManager &_ecm) override;

        // Documentation inherited
        public: void PostUpdate(
                    const UpdateInfo &_info,
                    const EntityComponentManager &_ecm) override;

        /// \brief Private data pointer
        private: std::unique_ptr<MimicJointPrivate> dataPtr;

      };
      }

}
}
}

#endif // IGNITION_GAZEBO_MIMIC_JOINT_PLUGIN_HH
