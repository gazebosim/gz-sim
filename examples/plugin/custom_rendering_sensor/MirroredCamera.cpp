/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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
 */

//needed in order to register plugins
#include <gz/plugin/Register.hh>

//header file
#include "MirroredCamera.hh"

//headers for other code used
#include </gz/sensors/CameraSensor.hh>
#include </gz/common/Image.hh>
#include <FreeImage.h>
#include </gz/sensors/RenderingSensor.hh>

using namespace custom;

//////////////////////////////////////////////////
void MirroredCamera::PostUpdate(const gz::sim::UpdateInfo &_info,
                            const gz::sim::EntityComponentManager &_ecm)
{
    //create a camera
    gz:sensors:CameraSensor camera;
    camera->CreateCamera();

    //flip image horizontally by accessing the bitmap pointer?
    FreeImage_FlipHorizontal(camera->image->*bitmap);
}

GZ_ADD_PLUGIN(custom_rendering_sensor::MirroredCamera, gz::sim::System,
        custom_rendering_sensor::MirroredCamera::ISystemPostUpdate)