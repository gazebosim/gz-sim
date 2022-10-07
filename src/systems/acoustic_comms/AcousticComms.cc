/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

/*
 * Development of this module has been funded by the Monterey Bay Aquarium
 * Research Institute (MBARI) and the David and Lucile Packard Foundation
 */
#include <unordered_map>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include "gz/sim/comms/Broker.hh"
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/Util.hh"
#include "AcousticComms.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Acoustic comms data class.
class AcousticComms::Implementation
{
  /// \brief Default max range for acoustic comms in metres.
  public: double maxRange = 1000.0;

  /// \brief Default speed of sound in air in metres/sec.
  public: double speedOfSound = 343.0;

  /// \brief Default collision time interval in sec.
  public: double collisionTimeInterval = 0;

  /// \brief Position of the transmitter at the time the message was
  /// sent, or first processed.
  public: std::unordered_map
          <std::shared_ptr<msgs::Dataframe>, math::Vector3d>
          poseSrcAtMsgTimestamp;

  /// \brief Map that holds data of the address of a receiver, and
  /// the timestamp of the last message recevied by it.
  public: std::unordered_map
          <std::string, std::chrono::duration<double>>
          lastMsgReceivedTime;
};

//////////////////////////////////////////////////
AcousticComms::AcousticComms()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void AcousticComms::Load(
    const Entity &_entity,
    std::shared_ptr<const sdf::Element> _sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  if (_sdf->HasElement("max_range"))
  {
    this->dataPtr->maxRange = _sdf->Get<double>("max_range");
  }
  if (_sdf->HasElement("speed_of_sound"))
  {
    this->dataPtr->speedOfSound = _sdf->Get<double>("speed_of_sound");
  }
  if (_sdf->HasElement("collision_time_interval"))
  {
    this->dataPtr->collisionTimeInterval =
      _sdf->Get<double>("collision_time_interval");
  }

  gzmsg << "AcousticComms configured with max range : " <<
    this->dataPtr->maxRange << " m and speed of sound : " <<
    this->dataPtr->speedOfSound << " m/s." << std::endl;
}

//////////////////////////////////////////////////
void AcousticComms::Step(
    const UpdateInfo &_info,
    const comms::Registry &_currentRegistry,
    comms::Registry &_newRegistry,
    EntityComponentManager &_ecm)
{
  // Initialize entity if needed.
  for (auto & [address, content] : _currentRegistry)
  {
    if (content.entity == kNullEntity)
    {
      auto entities = entitiesFromScopedName(content.modelName, _ecm);
      if (entities.empty())
        continue;

      auto entityId = *(entities.begin());
      if (entityId == kNullEntity)
        continue;

      _newRegistry[address].entity = entityId;
    }
  }

  for (auto & [address, content] : _currentRegistry)
  {
    // Reference to the outbound queue for this address.
    auto &outbound = content.outboundMsgs;

    // Is the source address bound?
    auto itSrc = _currentRegistry.find(address);
    bool srcAddressBound = itSrc != _currentRegistry.end();

    // Is the source address attached to a model?
    bool srcAddressAttachedToModel =
      srcAddressBound && itSrc->second.entity != kNullEntity;

    comms::DataQueue newOutbound;

    if (srcAddressAttachedToModel)
    {
      // All these messages need to be processed.
      for (auto &msg : outbound)
      {
        // Is the destination address bound?
        auto itDst = _currentRegistry.find(msg->dst_address());
        bool dstAddressBound = itDst != _currentRegistry.end();

        // Is the destination address attached to a model?
        bool dstAddressAttachedToModel =
          dstAddressBound && itDst->second.entity != kNullEntity;

        if (!dstAddressAttachedToModel)
          continue;

        // The plugin checks the distance travelled by the signal
        // so far. If it is more than the maxRange, it is dropped
        // and would never reach the destination.
        // If it has already reached the destination but not as far
        // as maxRange, it is processed.
        // If it has reached neither the destination nor the maxRange,
        // it is considered in transit.

        if (this->dataPtr->poseSrcAtMsgTimestamp.count(msg) == 0)
        {
          // This message is being processed for the first time.
          // Record the current position of the sender and use it
          // for distance calculations.
          this->dataPtr->poseSrcAtMsgTimestamp[msg] =
            worldPose(itSrc->second.entity, _ecm).Pos();
        }

        const auto& poseSrc =
          this->dataPtr->poseSrcAtMsgTimestamp[msg];

        // Calculate distance between the bodies.
        const auto poseDst = worldPose(itDst->second.entity, _ecm).Pos();
        const auto distanceToTransmitter = (poseSrc - poseDst).Length();

        // Calculate distance covered by the message.
        const std::chrono::steady_clock::time_point currTime(_info.simTime);
        const auto timeOfTransmission = msg->mutable_header()->stamp();

        const auto currTimestamp =
          std::chrono::nanoseconds(currTime.time_since_epoch());
        const auto packetTimestamp =
          std::chrono::seconds(timeOfTransmission.sec()) +
          std::chrono::nanoseconds(timeOfTransmission.nsec());

        const std::chrono::duration<double> deltaT =
          currTimestamp - packetTimestamp;
        const double distanceCoveredByMessage = deltaT.count() *
          this->dataPtr->speedOfSound;

        // Check the msgs that haven't exceeded the maxRange.
        if (distanceCoveredByMessage <= this->dataPtr->maxRange)
        {
          if (distanceCoveredByMessage >= distanceToTransmitter)
          {
            // This message has effectively reached the destination.
            bool receivedSuccessfully = false;

            // Check for time collision
            if (this->dataPtr->lastMsgReceivedTime.count(
                  msg->dst_address()) == 0)
            {
              // This is the first message received by this address.
              receivedSuccessfully = true;
            }
            else
            {
              // A previous msg was already received at this address.
              std::chrono::duration<double> timeGap = currTimestamp -
                this->dataPtr->lastMsgReceivedTime[msg->dst_address()];

              auto dropInterval = std::chrono::duration<double>(
                  this->dataPtr->collisionTimeInterval);

              if (timeGap >= dropInterval)
                receivedSuccessfully = true;
            }

            // This message needs to be processed.
            // Push the msg to inbound of the destination if receivedSuccessfully
            // is true, else it is dropped.
            if (receivedSuccessfully)
            {
              _newRegistry[msg->dst_address()].inboundMsgs.push_back(msg);
              // Update the receive time of the last msg for this address.
              this->dataPtr->lastMsgReceivedTime[msg->dst_address()] =
                currTimestamp;
            }

            // Stop keeping track of the position of its source.
            this->dataPtr->poseSrcAtMsgTimestamp.erase(msg);
          }
          else
          {
            // This message is still in transit, should be kept in the
            // outbound buffer of source and not moved to inbound of
            // the destination.
            newOutbound.push_back(msg);
          }
        }
        else
        {
          // This message exceeded the maxRange.
          // Stop keeping track of the position of its source.
          this->dataPtr->poseSrcAtMsgTimestamp.erase(msg);
        }
      }
    }

    // Clear the outbound queue, except for messages that were
    // in transit.
    _newRegistry[address].outboundMsgs = newOutbound;
  }
}

GZ_ADD_PLUGIN(AcousticComms,
              System,
              comms::ICommsModel::ISystemConfigure,
              comms::ICommsModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AcousticComms,
                          "gz::sim::systems::AcousticComms")
