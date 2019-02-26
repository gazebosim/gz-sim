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

#ifndef IGNITION_GAZEBO_SYNCMANAGER_HH
#define IGNITION_GAZEBO_SYNCMANAGER_HH

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <sdf/Element.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/network/NetworkRole.hh"
#include "ignition/transport/Node.hh"


namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // forward declaration
    class SimulationRunner;

    class SyncManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      /// \param[in] _useLevels Whether the simulationrunner is using levels.
      /// \param[in] _useDistSim Whether the simulationrunner is distributed
      /// simulation.
      public: SyncManager(SimulationRunner *_runner, bool _useLevels = false,
                  bool _useDistSim = false);

      public: void DistributePerformers();

      public: bool Sync();

      /// \brief Pointer to the simulation runner associated with the sync
      /// manager.
      private: SimulationRunner *const runner;

      /// \brief Entity of the world.
      private: Entity worldEntity{kNullEntity};

      private: ignition::transport::Node node;

      private: ignition::transport::Node::Publisher posePub;

      private: void OnPose(const ignition::msgs::Pose_V &_msg);

      private: std::mutex poseMutex;

      private: std::vector<ignition::msgs::Pose_V> poseMsgs;

      private: std::vector<Entity> performers;

      private: std::unordered_map<std::string, std::vector<Entity>> affinity;

      /// \brief Role of this network participant.
      private: NetworkRole role;

      /// \brief Flag whether to use levels or not.
      private: bool useLevels{false};

      /// \brief Flag whether distributed simulation is in use.
      private: bool useDistSim{false};

      /// \brief Entity Creator API.
      private: std::unique_ptr<SdfEntityCreator> entityCreator{nullptr};
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_SYNCMANAGER_HH
