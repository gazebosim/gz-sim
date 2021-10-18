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

%module worldpose
%{
#include "ignition/gazebo/components/Pose.hh"
%}

namespace ignition
{
  namespace gazebo
  {
    class WorldPose
    {
      /// \brief A component type that contains pose, ignition::math::Pose3d,
      /// information in world frame.
      using WorldPose = Component<ignition::math::Pose3d, class WorldPoseTag>;
      IGN_GAZEBO_REGISTER_COMPONENT(
          "ign_gazebo_components.WorldPose", WorldPose)

    };
  }
}
