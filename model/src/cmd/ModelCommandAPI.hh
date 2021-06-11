/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Entity.hh"
#include <ignition/gazebo/EntityComponentManager.hh>
#include <string>

extern "C"
{
    /// \brief Get the name of the world being used.
    std::string getWorldName();

    /// \brief Print the model pose information.
    void printPose(uint64_t entity,
        ignition::gazebo::EntityComponentManager &ecm);

    /// \brief Print the model links information.
    void printLinks(uint64_t entity,
        ignition::gazebo::EntityComponentManager &ecm, std::string link_name);

    /// \brief Print the model joints information.
    void printJoints(uint64_t entity,
        ignition::gazebo::EntityComponentManager &ecm, std::string joint_name);

    /// \brief Print the model bounding box information.
    void printBoundingBox(uint64_t entity,
        ignition::gazebo::EntityComponentManager &ecm);

    /// \brief External hook to get a list of available models.
    extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelList();

    /// \brief External hook to dump model information.
    extern "C" IGNITION_GAZEBO_VISIBLE void cmdModelInfo(
        const char *_model, int _pose, int _link, const char *_link_name,
        int _joint, const char *joint_name);
}
