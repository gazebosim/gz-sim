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

#include <gz/msgs/dataframe.pb.h>

#include <limits>
#include <list>
#include <random>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include <sdf/sdf.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "RFComms.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Parameters for simple log-normal fading model.
struct RangeConfiguration
{
  /// \brief Hard limit on range.
  double maxRange = 50.0;

  /// \brief Fading exponent.
  double fadingExponent = 2.5;

  /// \brief Received power at 1m (in dBm).
  double l0 = 40;

  /// \brief Standard deviation for received power.
  double sigma = 10;

  /// Output stream operator.
  /// \param[out] _oss Stream.
  /// \param[in] _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RangeConfiguration &_config)
  {
    _oss << "RF Configuration (range-based)" << std::endl
         << "-- max_range: " << _config.maxRange << std::endl
         << "-- fading_exponent: " << _config.fadingExponent << std::endl
         << "-- l0: " << _config.l0 << std::endl
         << "-- sigma: " << _config.sigma << std::endl;

    return _oss;
  }
};

/// \brief Radio configuration parameters.
///
/// Static parameters such as channel capacity and transmit power.
struct RadioConfiguration
{
  /// \brief Capacity of radio in bits-per-second.
  double capacity = 54000000;

  /// \brief Default transmit power in dBm. Default is 27dBm or 500mW.
  double txPower = 27;

  /// \brief Modulation scheme, e.g., QPSK (Quadrature Phase Shift Keyring).
  std::string modulation = "QPSK";

  /// \brief Noise floor of the radio in dBm.
  double noiseFloor = -90;

  /// Output stream operator.
  /// \param _oss Stream.
  /// \param _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RadioConfiguration &_config)
  {
    _oss << "Radio Configuration" << std::endl
         << "-- capacity: " << _config.capacity << std::endl
         << "-- tx_power: " << _config.txPower << std::endl
         << "-- noise_floor: " << _config.noiseFloor << std::endl
         << "-- modulation: " << _config.modulation << std::endl;

    return _oss;
  }
};

/// \brief Store radio state
///
/// Structure to hold radio state including the pose and book-keeping
/// necessary to implement bitrate limits.
struct RadioState
{
  /// \brief Timestamp of last update.
  double timeStamp;

  /// \brief Pose of the radio.
  gz::math::Pose3<double> pose;

  /// \brief Recent sent packet history.
  std::list<std::pair<double, uint64_t>> bytesSent;

  /// \brief Accumulation of bytes sent in an epoch.
  uint64_t bytesSentThisEpoch = 0;

  /// \brief Recent received packet history.
  std::list<std::pair<double, uint64_t>> bytesReceived;

  /// \brief Accumulation of bytes received in an epoch.
  uint64_t bytesReceivedThisEpoch = 0;

  /// \brief Name of the model associated with the radio.
  std::string name;
};

/// \brief Type for holding RF power as a Normally distributed random variable.
struct RFPower
{
  /// \brief Expected value of RF power.
  double mean;

  /// \brief Variance of RF power.
  double variance;

  /// \brief double operator.
  /// \return the RFPower as a double.
  operator double() const
  {
    return mean;
  }
};

/// \brief Private RFComms data class.
class gz::sim::systems::RFComms::Implementation
{
  /// \brief Attempt communication between two nodes.
  ///
  /// The radio configuration, transmitter and receiver state, and
  /// packet size are all used to compute the probability of successful
  /// communication (i.e., based on both SNR and bitrate
  /// limitations). This probability is then used to determine if the
  /// packet is successfully communicated.
  ///
  /// \param[in out] _txState Current state of the transmitter.
  /// \param[in out] _rxState Current state of the receiver.
  /// \param[in] _numBytes Size of the packet.
  /// \return std::tuple<bool, double> reporting if the packet should be
  /// delivered and the received signal strength (in dBm).
  public: std::tuple<bool, double> AttemptSend(RadioState &_txState,
                                               RadioState &_rxState,
                                               const uint64_t &_numBytes);

  /// \brief Convert from dBm to power.
  /// \param[in] _dBm Input in dBm.
  /// \return Power in watts (W).
  private: double DbmToPow(double _dBm) const;

  /// \brief Compute the bit error rate (BER).
  /// \param[in] _power Rx power (dBm).
  /// \param[in] _noise Noise value (dBm).
  /// \return Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER).
  private: double QPSKPowerToBER(double _power,
                                 double _noise) const;

  /// \brief Function to compute the pathloss between two antenna poses.
  /// \param[in] _txPower Tx power.
  /// \param[in] _txState Radio state of the transmitter.
  /// \param[in] _rxState Radio state of the receiver.
  /// \return The RFPower pathloss distribution of the two antenna poses.
  private: RFPower LogNormalReceivedPower(const double &_txPower,
                                          const RadioState &_txState,
                                          const RadioState &_rxState) const;

  /// \brief Range configuration.
  public: RangeConfiguration rangeConfig;

  /// \brief Radio configuration.
  public: RadioConfiguration radioConfig;

  /// \brief A map where the key is the address and the value its radio state.
  public: std::unordered_map<std::string, RadioState> radioStates;

  /// \brief Duration of an epoch (seconds).
  public: double epochDuration = 1.0;

  /// \brief Random device to seed random engine
  public: std::random_device rd{};

  /// \brief Random number generator.
  public: std::default_random_engine rndEngine{rd()};
};

/////////////////////////////////////////////
double RFComms::Implementation::DbmToPow(double _dBm) const
{
  return 0.001 * pow(10., _dBm / 10.);
}

////////////////////////////////////////////
double RFComms::Implementation::QPSKPowerToBER(
  double _power, double _noise) const
{
  return erfc(sqrt(_power / _noise));
}

/////////////////////////////////////////////
RFPower RFComms::Implementation::LogNormalReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange)
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = this->rangeConfig.l0 +
    10 * this->rangeConfig.fadingExponent * log10(kRange);

  return {_txPower - kPL, pow(this->rangeConfig.sigma, 2.)};
}

/////////////////////////////////////////////
std::tuple<bool, double> RFComms::Implementation::AttemptSend(
  RadioState &_txState, RadioState &_rxState, const uint64_t &_numBytes)
{
  double now = _txState.timeStamp;

  // Maintain running window of bytes sent over the last epoch, e.g., 1s.
  while (!_txState.bytesSent.empty() &&
         _txState.bytesSent.front().first <= now - this->epochDuration)
  {
    _txState.bytesSentThisEpoch -= _txState.bytesSent.front().second;
    _txState.bytesSent.pop_front();
  }

  // gzdbg << "Bytes sent: " <<  _txState.bytesSentThisEpoch << " + "
  //        << _numBytes << " = "
  //        << _txState.bytesSentThisEpoch + _numBytes << std::endl;

  // Compute prospective accumulated bits along with time window
  // (including this packet).
  auto bitsSent =
    static_cast<double>((_txState.bytesSentThisEpoch + _numBytes) * 8);

  // Check current epoch bitrate vs capacity and fail to send accordingly
  if (bitsSent > this->radioConfig.capacity * this->epochDuration)
  {
    gzwarn << "Bitrate limited: [" << _txState.name << "] " << bitsSent
            << " bits sent (limit: "
            << this->radioConfig.capacity * this->epochDuration << ")"
            << std::endl;
    return std::make_tuple(false, std::numeric_limits<double>::lowest());
  }

  // Record these bytes.
  _txState.bytesSent.push_back(std::make_pair(now, _numBytes));
  _txState.bytesSentThisEpoch += _numBytes;

  // Get the received power based on TX power and position of each node.
  auto rxPowerDist =
    this->LogNormalReceivedPower(this->radioConfig.txPower, _txState, _rxState);

  double rxPower = rxPowerDist.mean;
  if (rxPowerDist.variance > 0.0)
  {
    std::normal_distribution<> d{rxPowerDist.mean, sqrt(rxPowerDist.variance)};
    rxPower = d(this->rndEngine);
  }

  // Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER).
  double ber = this->QPSKPowerToBER(
    this->DbmToPow(rxPower), this->DbmToPow(this->radioConfig.noiseFloor));
  double packetDropProb =

    1.0 - exp(static_cast<double>(_numBytes) * log(1 - ber));

  // gzdbg << "TX power (dBm): " << this->radioConfig.txPower << "\n" <<
  //           "RX power (dBm): " << rxPower << "\n" <<
  //           "BER: " << ber << "\n" <<
  //           "# Bytes: " << _numBytes << "\n" <<
  //           "PER: " << packetDropProb << std::endl;

  double randDraw = gz::math::Rand::DblUniform();
  bool packetReceived = randDraw > packetDropProb;

  if (!packetReceived)
    return std::make_tuple(false, std::numeric_limits<double>::lowest());

  // Maintain running window of bytes received over the last epoch, e.g., 1s.
  while (!_rxState.bytesReceived.empty() &&
         _rxState.bytesReceived.front().first <= now - this->epochDuration)
  {
    _rxState.bytesReceivedThisEpoch -= _rxState.bytesReceived.front().second;
    _rxState.bytesReceived.pop_front();
  }

  // gzdbg << "bytes received: " << _rxState.bytesReceivedThisEpoch
  //        << " + " << _numBytes
  //       << " = " << _rxState.bytesReceivedThisEpoch + _numBytes << std::endl;

  // Compute prospective accumulated bits along with time window
  // (including this packet).
  auto bitsReceived =
    static_cast<double>((_rxState.bytesReceivedThisEpoch + _numBytes) * 8);

  // Check current epoch bitrate vs capacity and fail to send accordingly.
  if (bitsReceived > this->radioConfig.capacity * this->epochDuration)
  {
    gzwarn << "Bitrate limited: [" << _rxState.name << "] " <<  bitsReceived
           << " bits received (limit: "
           << this->radioConfig.capacity * this->epochDuration << ")"
           << std::endl;
    return std::make_tuple(false, std::numeric_limits<double>::lowest());
  }

  // Record these bytes.
  _rxState.bytesReceived.push_back(std::make_pair(now, _numBytes));
  _rxState.bytesReceivedThisEpoch += _numBytes;

  return std::make_tuple(true, rxPower);
}

//////////////////////////////////////////////////
RFComms::RFComms()
  : dataPtr(gz::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void RFComms::Load(const Entity &/*_entity*/,
    std::shared_ptr<const sdf::Element> _sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  if (_sdf->HasElement("range_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("range_config");

    this->dataPtr->rangeConfig.maxRange =
      elem->Get<double>("max_range", this->dataPtr->rangeConfig.maxRange).first;

    this->dataPtr->rangeConfig.fadingExponent =
      elem->Get<double>("fading_exponent",
        this->dataPtr->rangeConfig.fadingExponent).first;

    this->dataPtr->rangeConfig.l0 =
      elem->Get<double>("l0", this->dataPtr->rangeConfig.l0).first;

    this->dataPtr->rangeConfig.sigma =
      elem->Get<double>("sigma", this->dataPtr->rangeConfig.sigma).first;
  }

  if (_sdf->HasElement("radio_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("radio_config");

    this->dataPtr->radioConfig.capacity =
      elem->Get<double>("capacity", this->dataPtr->radioConfig.capacity).first;

    this->dataPtr->radioConfig.txPower =
      elem->Get<double>("tx_power", this->dataPtr->radioConfig.txPower).first;

    this->dataPtr->radioConfig.modulation =
      elem->Get<std::string>("modulation",
        this->dataPtr->radioConfig.modulation).first;

    this->dataPtr->radioConfig.noiseFloor =
      elem->Get<double>("noise_floor",
        this->dataPtr->radioConfig.noiseFloor).first;
  }

  gzdbg << "Range configuration:" << std::endl
         << this->dataPtr->rangeConfig << std::endl;

  gzdbg << "Radio configuration:" << std::endl
         << this->dataPtr->radioConfig << std::endl;
}

//////////////////////////////////////////////////
void RFComms::Step(
      const UpdateInfo &_info,
      const comms::Registry &_currentRegistry,
      comms::Registry &_newRegistry,
      EntityComponentManager &_ecm)
{
  // Update ratio states.
  for (auto & [address, content] : _currentRegistry)
  {
    // Associate entity if needed.
    if (content.entity == kNullEntity)
    {
      auto entities = sim::entitiesFromScopedName(content.modelName, _ecm);
      if (entities.empty())
        continue;

      auto entityId = *(entities.begin());
      if (entityId == kNullEntity)
        continue;

      _newRegistry[address].entity = entityId;

      enableComponent<components::WorldPose>(_ecm, entityId);
    }
    else
    {
      // Update radio state.
      const auto kPose = sim::worldPose(content.entity, _ecm);
      this->dataPtr->radioStates[address].pose = kPose;
      this->dataPtr->radioStates[address].timeStamp =
        std::chrono::duration<double>(_info.simTime).count();
      this->dataPtr->radioStates[address].name = content.modelName;
    }
  }

  for (auto & [address, content] : _currentRegistry)
  {
    // Reference to the outbound queue for this address.
    auto &outbound = content.outboundMsgs;

    // The source address needs to be attached to a robot.
    auto itSrc = this->dataPtr->radioStates.find(address);
    if (itSrc != this->dataPtr->radioStates.end())
    {
      // All these messages need to be processed.
      for (const auto &msg : outbound)
      {
        // The destination address needs to be attached to a robot.
        auto itDst = this->dataPtr->radioStates.find(msg->dst_address());
        if (itDst == this->dataPtr->radioStates.end())
          continue;

        auto [sendPacket, rssi] = this->dataPtr->AttemptSend(
          itSrc->second, itDst->second, msg->data().size());

        if (sendPacket)
        {
          // We create a copy of the outbound message because each destination
          // might have a different rssi value.
          auto inboundMsg = std::make_shared<msgs::Dataframe>(*msg);

          // Add rssi.
          auto *rssiPtr = inboundMsg->mutable_header()->add_data();
          rssiPtr->set_key("rssi");
          rssiPtr->add_value(std::to_string(rssi));

          _newRegistry[msg->dst_address()].inboundMsgs.push_back(inboundMsg);
        }
      }
    }

    // Clear the outbound queue.
    _newRegistry[address].outboundMsgs.clear();
  }
}

GZ_ADD_PLUGIN(RFComms,
                    gz::sim::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RFComms,
                          "gz::sim::systems::RFComms")
