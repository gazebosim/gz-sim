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

  /// \brief Default collision time interval per byte in sec.
  public: double collisionTimePerByte = 0;

  /// \brief Default collision time when packet is dropped
  /// in sec.
  public: double collisionTimePacketDrop = 0;

  /// \brief Position of the transmitter at the time the message was
  /// sent, or first processed.
  public: std::unordered_map
          <std::shared_ptr<msgs::Dataframe>, math::Vector3d>
          poseSrcAtMsgTimestamp;

  /// \brief Map that holds data of the address of a receiver,
  /// the timestamp, length of the last message recevied by it.
  public: std::unordered_map
          <std::string,
           std::tuple<std::chrono::duration<double>,
            std::chrono::duration<double>>>
          lastMsgReceivedInfo;

  /// \brief This method simulates the propagation model,
  /// and returns false if the packet was dropped, otherwise true.
  public: bool propagationModel(double _distToSource,
                                int _numBytes);

  /// \brief Source power in Watts.
  /// \ref https://www.sciencedirect.com/topics/
  /// computer-science/underwater-acoustic-signal
  public: double sourcePower = 2000;

  /// \brief Ratio of the noise intensity at the
  /// receiver to the same reference intensity used for source level.
  public: double noiseLevel = 1;

  /// \brief Information rate that can be transmitted over a given
  /// bandwidth in a specific communication system, in (bits/sec)/Hz.
  /// \ref: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5514747/
  public: double spectralEfficiency = 7.0;

  /// \brief Flag to store if the propagation model should be used.
  public: bool usePropagationModel = false;

  /// \brief Seed value for random sampling.
  public: unsigned int seed = 0;
};

//////////////////////////////////////////////////
bool AcousticComms::Implementation::propagationModel(
  double _distToSource,
  int _numBytes
)
{
  // From https://www.mathworks.com/help/phased/ug/sonar-equation.html
  // SNR = SL - TL - (NL - DI)
  // SNR : Signal to noise ratio.
  // SL : Source level. Ratio of the transmitted intensity from
  //      the source to a reference intensity (1 m from source),
  //      converted to dB.
  // TL : Transmission loss (dB)
  // NL : Noise level.

  // The constant 170.8 comes from reference intensity measured
  // 1m from the source.
  double sl = 170.8 + 10 * std::log10(this->sourcePower);
  double tl = 20 * std::log10(_distToSource);

  // Calculate SNR.
  auto snr = sl - tl - this->noiseLevel;

  // References : https://www.montana.edu/aolson/ee447/EB%20and%20NO.pdf
  // https://en.wikipedia.org/wiki/Eb/N0
  auto EbByN0 = snr / this->spectralEfficiency;

  // Bit error rate calculation using BPSK.
  // Reference : https://www.gaussianwaves.com/2012/07/
  // intuitive-derivation-of-performance-of-an-optimum-
  // bpsk-receiver-in-awgn-channel/
  // Reference : https://unetstack.net/handbook/unet-handbook_modems
  // _and_channel_models.html
  auto ber = 0.5 * std::erfc(EbByN0);

  // Calculate if the packet was dropped.
  double packetDropProb =
    1.0 - std::exp(static_cast<double>(_numBytes) *
                   std::log(1 - ber));

  gz::math::Rand::Seed(this->seed);
  double randDraw = gz::math::Rand::DblUniform();
  return randDraw > packetDropProb;
}

//////////////////////////////////////////////////
AcousticComms::AcousticComms()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void AcousticComms::Load(
    const Entity &/*_entity*/,
    std::shared_ptr<const sdf::Element> _sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  if (_sdf->HasElement("max_range"))
  {
    this->dataPtr->maxRange = _sdf->Get<double>("max_range");
  }
  if (_sdf->HasElement("speed_of_sound"))
  {
    this->dataPtr->speedOfSound = _sdf->Get<double>("speed_of_sound");
  }
  if (_sdf->HasElement("collision_time_per_byte"))
  {
    this->dataPtr->collisionTimePerByte =
      _sdf->Get<double>("collision_time_per_byte");
  }

  if (_sdf->HasElement("propagation_model"))
  {
    this->dataPtr->usePropagationModel = true;
    sdf::ElementPtr propElement = _sdf->Clone()->
                                   GetElement("propagation_model");
    this->dataPtr->sourcePower = propElement->Get<double>("source_power");
    this->dataPtr->noiseLevel = propElement->Get<double>("noise_level");
    this->dataPtr->spectralEfficiency =
      propElement->Get<double>("spectral_efficiency");
    this->dataPtr->seed = propElement->Get<int>("seed");
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
            if (this->dataPtr->lastMsgReceivedInfo.count(
                  msg->dst_address()) == 0)
            {
              // This is the first message received by this address.
              receivedSuccessfully = true;
            }
            else
            {
              // A previous msg was already received at this address.
              // time gap = current time - time at which last msg was received.
              std::chrono::duration<double> timeGap = currTimestamp -
                std::get<0>(this->dataPtr->lastMsgReceivedInfo[
                            msg->dst_address()]);

              // drop interval = collision time interval per byte *
              //                 length of last msg received.
              auto dropInterval = std::chrono::duration<double>(
                  std::get<1>(this->dataPtr->lastMsgReceivedInfo[
                              msg->dst_address()]));

              if (timeGap >= dropInterval)
                receivedSuccessfully = true;
            }

            // Packet has survived collisions, check if the propagation model
            // should be run on this packet.
            if (this->dataPtr->usePropagationModel)
            {
              receivedSuccessfully = receivedSuccessfully &&
                this->dataPtr->propagationModel(distanceCoveredByMessage,
                                                msg->data().length());
            }

            // This message needs to be processed.
            // Push the msg to inbound of the destination if
            // receivedSuccessfully is true, else it is dropped.
            if (receivedSuccessfully)
            {
              _newRegistry[msg->dst_address()].inboundMsgs.push_back(msg);
              // Update the (receive time, length of msg) tuple
              // for the last msg for this address.
              auto blockingTime = std::chrono::duration<double>(
                this->dataPtr->collisionTimePerByte *
                msg->data().length());

              this->dataPtr->lastMsgReceivedInfo[msg->dst_address()] =
                std::make_tuple(currTimestamp, blockingTime);
            }
            else
            {
              // Packet was dropped due to collision.
              auto blockingTime = std::chrono::duration<double>(
                  this->dataPtr->collisionTimePacketDrop
                );

              std::get<1>(this->dataPtr->lastMsgReceivedInfo[
                          msg->dst_address()]) += blockingTime;
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
