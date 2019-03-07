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
#include <string>
#include <unordered_map>
#include <vector>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/transport/Node.hh"

#include "network/NetworkRole.hh"

namespace ignition
{
  namespace gazebo
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
    //
    // forward declaration
    class SimulationRunner;

    /// \brief Used to manage syncronization between simulation primary and
    /// simulation secondaries.
    ///
    /// Utilizes the concept of "performers" introduced in the level manager.
    /// Each performer has an affinity, which is the mapping between the
    /// performer and the distributed simulation secondary. The distributed
    /// simulation primary does not have any performers of itself, but
    /// purely manages the distribution of performers to the secondaries.
    ///
    /// The SyncManager will also attach components to the performer entity to
    /// manage the distribution. The first is PerformerAffinity, to note the
    /// secondary that each performer is associated with. The second is
    /// PerformerActive, which marks whether the performer is active on this
    /// secondary.
    class SyncManager
    {
      /// \brief Constructor
      /// \param[in] _runner A pointer to the simulationrunner that owns this
      public: explicit SyncManager(SimulationRunner *_runner);

      /// \brief Distribute performer affinity to the secondaries in the
      /// distributed simulation environment.
      public: void DistributePerformers();

      /// \brief Syncronize state between primary and secondary
      /// EntityComponentManagers
      public: bool Sync();

      /// \brief Pointer to the simulation runner associated with the sync
      /// manager.
      private: SimulationRunner *const runner;

      /// \brief Callback for when pose syncronization is received.
      /// \param[in] _msg Message with vector of incoming pose updates
      /// \TODO(mjcarroll) to be replaced with ECM sync.
      private: void OnPose(const ignition::msgs::Pose_V &_msg);

      /// \brief Ignition transport communication node
      private: ignition::transport::Node node;

      /// \brief Publisher for managed perfomers
      /// \TODO(mjcarroll) - Update this to utilize ECM sync
      private: ignition::transport::Node::Publisher posePub;

      /// \brief Mutex to protect collection of incoming pose messages
      private: std::mutex poseMutex;

      /// \brief Collection of incoming pose update messages
      private: std::vector<ignition::msgs::Pose_V> poseMsgs;

      /// \brief Collection of performers associated with a secondary
      private: std::vector<Entity> performers;

      /// \brief Map of secondary identifiers and associated performer entities.
      private: std::unordered_map<std::string, std::vector<Entity>> affinity;

      /// \brief Role of this network participant.
      private: NetworkRole role;

      /// \brief Entity Creator API.
      private: std::unique_ptr<SdfEntityCreator> entityCreator{nullptr};
    };
    }
  }
}
#endif  // IGNITION_GAZEBO_SYNCMANAGER_HH
