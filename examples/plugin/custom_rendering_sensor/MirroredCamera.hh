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

#ifndef GAZEBO_MIRROREDCAMERA_HH
#define GAZEBO_MIRROREDCAMERA_HH

#include <gz/sim/System.hh>

#include </gz/sensors/CameraSensor.hh>
#include </gz/common/Image.hh>
#include <FreeImage.h>
#include </gz/sensors/RenderingSensor.hh>

namespace custom
{
    /// \brief Attempt at creating a custom rendering sensor (in this case,
    /// a camera sensor with its image flipped horizontally) as an example
    /// for future developers)
    class MirroredCamera:
            public gz::sim::System,
            public gz::sim::ISystemPostUpdate

    /// \brief Creates a camera sensor, then calls the FlipHorizontal method
    /// from the FreeImage library on the the pointer to the bitmap data
    /// from the camera sensor's image field
    {
        void PostUpdate(const gz::sim::UpdateInfo &_info,
                        const gz::sim::EntityComponentManager &_ecm)
    };
}

#endif //GAZEBO_MIRROREDCAMERA_HH
