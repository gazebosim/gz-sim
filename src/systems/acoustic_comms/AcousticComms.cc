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

#include <cmath>

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

class AcousticComms::Implementation
{
  // Default max range for acoustic comms in metres.
  public: double maxRange = 500;
  
  // Default speed of sound in air (m/s).
  public: double speedOfSound = 343;

  public: double DistanceBetweenBodies(
              math::Vector3<double> _src,
              math::Vector3<double> _dst);
};

double AcousticComms::Implementation::DistanceBetweenBodies(
    math::Vector3<double> _src, math::Vector3<double> _dst)
{
  double x = _src.X() - _dst.X();
  double y = _src.Y() - _dst.Y();
  double z = _src.Z() - _dst.Z();
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

AcousticComms::AcousticComms()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

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
    this->dataPtr->maxRange = _sdf->Get<double>("speed_of_sound");
  }
}

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

        // Calculate distance between the bodies.
        auto poseSrc = worldPose(itSrc->second.entity, _ecm).Pos();
        auto poseDst = worldPose(itDst->second.entity, _ecm).Pos();

        auto distanceToTransmitter =
          this->dataPtr->DistanceBetweenBodies(poseSrc,
              poseDst);

        // Calculate distance covered by the message.
        std::chrono::steady_clock::time_point currTime(_info.simTime);
        auto timeOfTransmission = msg->mutable_header()->stamp();

        double currTimestamp =
          currTime.time_since_epoch().count() / 1000000000.0;
        double packetTimestamp =
          static_cast<double>(timeOfTransmission.sec()) +
          static_cast<double>(timeOfTransmission.nsec()) / 1000000000.0;

        double deltaT = currTimestamp - packetTimestamp;
        double distanceCoveredByMessage = deltaT * this->dataPtr->speedOfSound;

        // Only check msgs that haven't exceeded the maxRange.
        if (distanceCoveredByMessage <= this->dataPtr->maxRange)
        {
          if (distanceCoveredByMessage >= distanceToTransmitter)
          {
            // This message needs to be processed.
            _newRegistry[msg->dst_address()].inboundMsgs.push_back(msg);
          }
          else
          {
            // This message is still in transit, should be kept in the
            // outbound buffer of source and not moved to inbound of
            // the destination.
            newOutbound.push_back(msg);
          }
        }
      }
    }

    // Clear the outbound queue, leaving the message in transit.
    _newRegistry[address].outboundMsgs = newOutbound;
  }
}

GZ_ADD_PLUGIN(AcousticComms,
              System,
              comms::ICommsModel::ISystemConfigure,
              comms::ICommsModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AcousticComms,
                          "gz::sim::systems::AcousticComms")
