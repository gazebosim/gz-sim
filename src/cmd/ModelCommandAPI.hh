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

#include "gz/sim/Export.hh"

/// \brief External hook to get a list of available models.
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelList();

/// \brief External hook to dump model information.
/// \param[in] _modelName Model name.
/// \param[in] _pose --pose option.
/// \param[in] _link_name Link name.
/// \param[in] _joint_name Joint name.
extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelInfo(
    const char *_modelName, int _pose, const char *_linkName,
    const char *_jointName);

