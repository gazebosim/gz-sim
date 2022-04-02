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

#include <sdf/sdf.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/gazebo/comms/MsgManager.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "WirelessComms.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Parameters for simple log-normal fading model.
struct RFConfiguration
{
  /// \brief Hard limit on range.
  double maxRange = 50.0;
  /// \brief Fading exponent.
  double fadingExponent = 2.5;
  /// \brief Received power at 1m (in dBm).
  double l0 = 40;
  /// \beief Standard deviation for received power.
  double sigma = 10;

  /// Output stream operator.
  /// @param[out] _oss Stream.
  /// @param[in] _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RFConfiguration &_config)
  {
    _oss << "RF Configuration (range-based)" << std::endl
         << "-- max_range: " << _config.maxRange << std::endl
         << "-- fading_exponent: " << _config.fadingExponent << std::endl
         << "-- L0: " << _config.l0 << std::endl
         << "-- sigma: " << _config.sigma << std::endl;

    return _oss;
  }
};

/// \brief Radio configuration parameters.
///
/// In addition to static parameters such as channel capacity and
/// default transmit power, this structure holds a function which can
/// be called to compute the pathloss between two antenna poses.
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
  /// \brief Function handle for computing pathloss.
  //rf_interface::pathloss_function pathloss_f;

  /// Output stream operator.
  /// @param _oss Stream.
  /// @param _config configuration to output.
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
  double updateStamp;

  /// \brief Pose of the radio.
  ignition::math::Pose3<double> pose;

  /// \brief Recent sent packet history.
  std::list<std::pair<double, uint64_t>> bytesSent;

  /// \brief Accumulation of bytes sent in an epoch.
  uint64_t bytesSentThisEpoch = 0;

  /// \brief Recent received packet history.
  std::list<std::pair<double, uint64_t>> bytesReceived;

  /// \brief Accumulation of bytes received in an epoch.
  uint64_t bytesReceivedThisEpoch = 0;

  /// \brief Isotropic antenna gain.
  double antennaGain;
};

/// \brief Type for holding RF power as a Normally distributed random variable.
struct RFPower
{
  /// \brief Expected value of RF power.
  double mean;

  /// \brief Variance of RF power.
  double variance;

  /// \brief ToDo.
  operator double() { return mean; }
};

/// \brief Private WirelessComms data class.
class ignition::gazebo::systems::WirelessComms::Implementation
{
  /// \brief Range configuration.
  public: RFConfiguration rangeConfig;

  /// \brief Radio configuration.
  public: RadioConfiguration radioConfig;

  /// \brief A map where the key is the model name and the value is the
  /// Model instance associated.
  public: std::unordered_map<std::string, gazebo::Link> links;
};

//////////////////////////////////////////////////
WirelessComms::WirelessComms()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
WirelessComms::~WirelessComms()
{
  // cannot use default destructor because of dataPtr
}

//////////////////////////////////////////////////
void WirelessComms::Load(const Entity &/*_entity*/,
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

  igndbg << "Range configuration:" << std::endl
         << this->dataPtr->rangeConfig << std::endl;

  igndbg << "Radio configuration:" << std::endl
         << this->dataPtr->radioConfig << std::endl;
}

//////////////////////////////////////////////////
void WirelessComms::Step(
      const UpdateInfo &/*_info*/,
      const comms::Registry &_currentRegistry,
      comms::Registry &_newRegistry,
      EntityComponentManager &_ecm)
{
  // Make sure that all addresses have its corresponding Link initialized.
  for (auto & [address, content] : _currentRegistry)
  {
    std::string linkName = content.modelName;
    if (this->dataPtr->links.find(address) != this->dataPtr->links.end())
      continue;

    auto entities = gazebo::entitiesFromScopedName(linkName, _ecm);
    if (entities.empty())
      continue;

    auto entityId = *(entities.begin());
    this->dataPtr->links[address] = gazebo::Link(entityId);
    enableComponent<components::WorldPose>(_ecm, entityId);
  }

  for (auto & [address, content] : _currentRegistry)
  {
    // Reference to the outbound queue for this address.
    auto &outbound = content.outboundMsgs;

    // All these messages need to be processed.
    for (auto &msg : outbound)
    {
      igndbg << "Pose src: "
             << *(this->dataPtr->links[msg->src_address()].WorldPose(_ecm))
             << std::endl;

      igndbg << "Pose dst: "
             << *(this->dataPtr->links[msg->dst_address()].WorldPose(_ecm))
             << std::endl;

      _newRegistry[msg->dst_address()].inboundMsgs.push_back(msg);
    }

    // Clear the outbound queue.
    _newRegistry[address].outboundMsgs.clear();
  }
}

IGNITION_ADD_PLUGIN(WirelessComms,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WirelessComms,
                          "ignition::gazebo::systems::WirelessComms")
