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

#include <google/protobuf/message.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity_factory.pb.h>

#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/sensors/Noise.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"

#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocitySeed.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Wind.hh"
#include "ignition/gazebo/components/WindMode.hh"

#include "ignition/gazebo/Link.hh"

#include "WindEffects.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private WindEffects data class.
class ignition::gazebo::systems::WindEffectsPrivate
{
  /// \brief Initialize the system.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  /// \param[in] _sdf Pointer to sdf::Element that contains configuration
  /// parameters for the system.
  public: void Load(EntityComponentManager &_ecm,
                    const std::shared_ptr<const sdf::Element> &_sdf);

  /// \brief Create subscribed topics and services.
  /// \param[in] _worldName Name of world entity.
  public: void SetupTransport(const std::string &_worldName);

  /// \brief Calculate and update the wind velocity component.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void UpdateWindVelocity(const UpdateInfo &_info,
                                  EntityComponentManager &_ecm);

  /// \brief Calculate and apply forces on links affected by wind.
  /// \param[in] _info Simulation update info.
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void ApplyWindForce(const UpdateInfo &_info,
                              EntityComponentManager &_ecm);

  /// \brief Callback for topic for setting the wind seed velocity and enabling
  /// this system.
  /// \param[in] _msg msgs::Wind message.
  public: void OnWindMsg(const msgs::Wind &_msg);

  /// \brief Process commands received from transport
  /// \param[in] _ecm Mutable reference to the EntityComponentManager.
  public: void ProcessCommandQueue(EntityComponentManager &_ecm);

  /// \brief Callback for servicing requests.
  /// \param[in] _msg msgs::Wind message.
  public: bool WindInfoService(msgs::Wind &_msg);

  /// \brief World entity to which this system is attached.
  public: Entity worldEntity;

  /// \brief Wind entity on which this sytem operates. There should only be one
  /// of these currently.
  public: Entity windEntity;

  /// \brief Time for wind to rise (time constant).
  public: double characteristicTimeForWindRise{1.0};

  /// \brief Wind amplitude.
  public: double magnitudeSinAmplitudePercent{0.0};

  /// \brief Wind period.
  public: double magnitudeSinPeriod{1.0};

  /// \brief Time for wind to change direction (time constant).
  public: double characteristicTimeForWindOrientationChange{1.0};

  /// \brief Orientation amplitude.
  public: double orientationSinAmplitude{0.0};

  /// \brief Orientation period.
  public: double orientationSinPeriod{1.0};

  /// \brief Mean of the magnitude.
  public: double magnitudeMean{0.0};

  /// \brief Mean of the direction. This is an optional so that it can be
  /// initialized with the first direction value encountered. If we don't do
  /// this, the filter's delay will cause the wind direction to always start
  /// with a non-zero component in the x-direction even if the seed velocity has
  /// no velocity in the x-direction.
  public: std::optional<double> directionMean;

  /// \brief Time for wind to rise.
  public: double characteristicTimeForWindRiseVertical{1.0};

  /// \brief Mean of the magnitude.
  public: double magnitudeMeanVertical{0.0};

  /// \brief The scaling factor to approximate wind as force on a mass.
  public: double forceApproximationScalingFactor{1.0};

  /// \brief Noise added to magnitude.
  public: sensors::NoisePtr noiseMagnitude;

  /// \brief Noise added to direction.
  public: sensors::NoisePtr noiseDirection;

  /// \brief Noise added to Z axis.
  public: sensors::NoisePtr noiseVertical;

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Set during Load to true if the configuration for the plugin is
  /// valid.
  public: bool validConfig{false};

  /// \brief Whether links have been initialized. Initialization involves
  /// creating components this system needs on links that are affected by wind.
  public: bool linksInitialized{false};

  /// \brief Mutex to protect currWindVelSeed and windEnabled
  public: std::mutex windInfoMutex;

  /// \brief Wind message queue
  public: std::vector<msgs::Wind> windCmdQueue;

  /// \brief Current wind velocity seed and global enable/disable state.
  /// This is set by a transport message.
  public: msgs::Wind currentWindInfo;
};

/////////////////////////////////////////////////
void WindEffectsPrivate::Load(EntityComponentManager &_ecm,
                              const std::shared_ptr<const sdf::Element> &_sdf)
{
  this->windEntity = _ecm.EntityByComponents(components::Wind());

  if (_sdf->HasElement("horizontal"))
  {
    auto sdfHoriz = _sdf->GetElementImpl("horizontal");

    if (sdfHoriz->HasElement("magnitude"))
    {
      auto sdfMag = sdfHoriz->GetElementImpl("magnitude");

      if (sdfMag->HasElement("time_for_rise"))
      {
        this->characteristicTimeForWindRise =
            sdfMag->Get<double>("time_for_rise");

        if (std::fabs(this->characteristicTimeForWindRise) < 1e-6)
        {
          ignerr << "Please set <horizontal><magnitude><time_for_rise> to a "
                 << "value greater than 0" << std::endl;
          return;
        }
      }

      if (sdfMag->HasElement("sin"))
      {
        auto sdfMagSin = sdfMag->GetElementImpl("sin");

        if (sdfMagSin->HasElement("amplitude_percent"))
        {
          this->magnitudeSinAmplitudePercent =
              sdfMagSin->Get<double>("amplitude_percent");
        }

        if (sdfMagSin->HasElement("period"))
        {
          this->magnitudeSinPeriod = sdfMagSin->Get<double>("period");
        }
      }

      if (sdfMag->HasElement("noise"))
      {
        this->noiseMagnitude = sensors::NoiseFactory::NewNoiseModel(
            sdfMag->GetElementImpl("noise"));
      }
    }

    if (sdfHoriz->HasElement("direction"))
    {
      auto sdfDir = sdfHoriz->GetElementImpl("direction");

      if (sdfDir->HasElement("time_for_rise"))
      {
        this->characteristicTimeForWindOrientationChange =
            sdfDir->Get<double>("time_for_rise");

        if (std::fabs(this->characteristicTimeForWindOrientationChange) < 1e-6)
        {
          ignerr << "Please set <horizontal><direction><time_for_rise> to a "
                 << "value greater than 0" << std::endl;
          return;
        }
      }

      if (sdfDir->HasElement("sin"))
      {
        auto sdfDirSin = sdfDir->GetElementImpl("sin");

        if (sdfDirSin->HasElement("amplitude"))
        {
          this->orientationSinAmplitude = sdfDirSin->Get<double>("amplitude");
        }

        if (sdfDirSin->HasElement("period"))
        {
          this->orientationSinPeriod = sdfDirSin->Get<double>("period");
        }
      }

      if (sdfDir->HasElement("noise"))
      {
        this->noiseDirection = sensors::NoiseFactory::NewNoiseModel(
            sdfDir->GetElementImpl("noise"));
      }
    }
  }

  if (_sdf->HasElement("vertical"))
  {
    auto sdfVert = _sdf->GetElementImpl("vertical");

    if (sdfVert->HasElement("time_for_rise"))
    {
      this->characteristicTimeForWindRiseVertical =
          sdfVert->Get<double>("time_for_rise");

      if (std::fabs(this->characteristicTimeForWindRiseVertical) < 1e-6)
      {
        ignerr << "Please set <horizontal><magnitude><time_for_rise> to a "
               << "value greater than 0" << std::endl;
        return;
      }
    }

    if (sdfVert->HasElement("noise"))
    {
      this->noiseVertical = sensors::NoiseFactory::NewNoiseModel(
          sdfVert->GetElementImpl("noise"));
    }
  }

  if (_sdf->HasElement("force_approximation_scaling_factor"))
  {
    sdf::ElementPtr sdfForceApprox =
        _sdf->GetElementImpl("force_approximation_scaling_factor");

    this->forceApproximationScalingFactor = sdfForceApprox->Get<double>();
  }

  // If the forceApproximationScalingFactor is very small don't update.
  // It doesn't make sense to be negative, that would be negative wind drag.
  if (std::fabs(this->forceApproximationScalingFactor) < 1e-6)
  {
    ignerr << "Please set <force_approximation_scaling_factor> to a value "
           << "greater than 0" << std::endl;
    return;
  }


  this->validConfig = true;
}

//////////////////////////////////////////////////
void WindEffectsPrivate::SetupTransport(const std::string &_worldName)
{
  // Wind seed velocity topic
  this->node.Subscribe("/world/" + _worldName + "/wind",
                       &WindEffectsPrivate::OnWindMsg, this);

  // Wind info service
  this->node.Advertise("/world/" + _worldName + "/wind_info",
                       &WindEffectsPrivate::WindInfoService, this);
}

//////////////////////////////////////////////////
void WindEffectsPrivate::UpdateWindVelocity(const UpdateInfo &_info,
                                            EntityComponentManager &_ecm)
{
  IGN_PROFILE("WindEffectsPrivate::UpdateWindVelocity");
  double period = std::chrono::duration<double>(_info.dt).count();
  double simTime = std::chrono::duration<double>(_info.simTime).count();
  double kMag = period / this->characteristicTimeForWindRise;
  double kMagVertical = period / this->characteristicTimeForWindRiseVertical;
  double kDir = period / this->characteristicTimeForWindOrientationChange;

  auto windLinVelSeed =
      _ecm.Component<components::WorldLinearVelocitySeed>(this->windEntity);
  auto windLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->windEntity);

  if (!windLinVelSeed || !windLinVel)
    return;

  double seedXYMagnitude =
        std::sqrt(windLinVelSeed->Data().X() * windLinVelSeed->Data().X() +
                  windLinVelSeed->Data().Y() * windLinVelSeed->Data().Y());

  // Compute magnitude
  this->magnitudeMean =
      (1.0 - kMag) * this->magnitudeMean + kMag * seedXYMagnitude;

  double magnitude = this->magnitudeMean;

  this->magnitudeMeanVertical =
      (1. - kMagVertical) * this->magnitudeMeanVertical +
      kMagVertical * windLinVelSeed->Data().Z();

  magnitude += this->magnitudeSinAmplitudePercent * this->magnitudeMean *
               std::sin(2 * IGN_PI * simTime / this->magnitudeSinPeriod);

  if (this->noiseMagnitude)
  {
    magnitude = this->noiseMagnitude->Apply(magnitude);
  }

  // Compute horizontal direction
  double direction =
      IGN_RTOD(atan2(windLinVelSeed->Data().Y(), windLinVelSeed->Data().X()));

  if (!this->directionMean)
  {
    this->directionMean = direction;
  }
  else
  {
    this->directionMean =
        (1.0 - kDir) * this->directionMean.value() + kDir * direction;
  }

  direction = this->directionMean.value();

  direction += this->orientationSinAmplitude *
               std::sin(2 * IGN_PI * simTime / this->orientationSinPeriod);

  if (this->noiseDirection)
    direction = this->noiseDirection->Apply(direction);

  // Apply wind velocity
  ignition::math::Vector3d windVel;
  windVel.X(magnitude * std::cos(IGN_DTOR(direction)));
  windVel.Y(magnitude * std::sin(IGN_DTOR(direction)));

  if (this->noiseVertical)
  {
    windVel.Z(this->noiseVertical->Apply(this->magnitudeMeanVertical));
  }
  else
  {
    windVel.Z(this->magnitudeMeanVertical);
  }

  // Update component
  windLinVel->Data() = windVel;
}

//////////////////////////////////////////////////
void WindEffectsPrivate::ApplyWindForce(const UpdateInfo &,
                                        EntityComponentManager &_ecm)
{
  IGN_PROFILE("WindEffectsPrivate::ApplyWindForce");
  auto windVel =
      _ecm.Component<components::WorldLinearVelocity>(this->windEntity);
  if (!windVel)
    return;

  Link link;

  _ecm.Each<components::Link, components::Inertial, components::WindMode,
            components::WorldLinearVelocity>(
      [&](const Entity &_entity,
          components::Link *,
          components::Inertial *_inertial,
          components::WindMode *_windMode,
          components::WorldLinearVelocity *_linkVel) -> bool
      {
        // Skip links for which the wind is disabled
        if (!_windMode->Data())
        {
          return true;
        }

        link.ResetEntity(_entity);

        math::Vector3d windForce = _inertial->Data().MassMatrix().Mass() *
                                   this->forceApproximationScalingFactor *
                                   (windVel->Data() - _linkVel->Data());

        // Apply force at center of mass
        link.AddWorldForce(_ecm, windForce);

        return true;
      });
}


//////////////////////////////////////////////////
void WindEffectsPrivate::OnWindMsg(const msgs::Wind &_msg)
{
  std::lock_guard<std::mutex> lock(this->windInfoMutex);
  this->windCmdQueue.push_back(_msg);
}

//////////////////////////////////////////////////
void WindEffectsPrivate::ProcessCommandQueue(EntityComponentManager &_ecm)
{
  std::lock_guard lock(this->windInfoMutex);
  if (this->windCmdQueue.size() > 0)
  {
    this->currentWindInfo.CopyFrom(this->windCmdQueue.back());
    this->windCmdQueue.pop_back();

    // Also update the seed velocity component
    auto windLinVelSeed =
        _ecm.Component<components::WorldLinearVelocitySeed>(this->windEntity);

    if (windLinVelSeed)
    {
      windLinVelSeed->Data() =
          msgs::Convert(this->currentWindInfo.linear_velocity());
    }
    else
    {
      _ecm.CreateComponent(this->windEntity,
                           components::WorldLinearVelocitySeed(msgs::Convert(
                               this->currentWindInfo.linear_velocity())));
    }
  }
}

//////////////////////////////////////////////////
bool WindEffectsPrivate::WindInfoService(msgs::Wind &_msg)
{
  // Update current wind velocity
  std::lock_guard<std::mutex> lock(this->windInfoMutex);
  _msg.CopyFrom(this->currentWindInfo);
  return true;
}

//////////////////////////////////////////////////
WindEffects::WindEffects() : System(),
    dataPtr(std::make_unique<WindEffectsPrivate>())
{
}

//////////////////////////////////////////////////
WindEffects::~WindEffects() = default;

//////////////////////////////////////////////////
void WindEffects::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->dataPtr->worldEntity = _entity;

  this->dataPtr->Load(_ecm, _sdf);

  if (this->dataPtr->validConfig)
  {
    auto worldName =
        _ecm.Component<components::Name>(this->dataPtr->worldEntity);
    if (worldName)
    {
      this->dataPtr->SetupTransport(worldName->Data());
    }

    // By default, the wind system is enabled.
    {
      std::lock_guard lock(this->dataPtr->windInfoMutex);
      this->dataPtr->currentWindInfo.set_enable_wind(true);
    }

    auto windLinVelSeed = _ecm.Component<components::WorldLinearVelocitySeed>(
        this->dataPtr->windEntity);

    if (windLinVelSeed)
    {
      // Update current wind velocity seed
      std::lock_guard lock(this->dataPtr->windInfoMutex);
      msgs::Set(this->dataPtr->currentWindInfo.mutable_linear_velocity(),
                windLinVelSeed->Data());
    }
  }
}

//////////////////////////////////////////////////
void WindEffects::PreUpdate(const UpdateInfo &_info,
                            EntityComponentManager &_ecm)
{
  IGN_PROFILE("WindEffects::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Process commands
  this->dataPtr->ProcessCommandQueue(_ecm);

  if (this->dataPtr->validConfig)
  {
    if (!this->dataPtr->linksInitialized)
    {
      _ecm.Each<components::Link, components::WindMode>(
          [&](const Entity &_entity, components::Link *,
              components::WindMode *_windMode) -> bool
          {
            if (_windMode->Data())
            {
              // Create a WorldLinearVelocity component on the link so that
              // physics can populate it
              if (!_ecm.Component<components::WorldLinearVelocity>(_entity))
              {
                _ecm.CreateComponent(_entity,
                                     components::WorldLinearVelocity());
              }
              if (!_ecm.Component<components::WorldPose>(_entity))
              {
                _ecm.CreateComponent(_entity, components::WorldPose());
              }
            }
            return true;
          });

      this->dataPtr->linksInitialized = true;
    }
    else
    {
      if (_info.paused)
        return;

      if (!this->dataPtr->currentWindInfo.enable_wind())
        return;

      this->dataPtr->UpdateWindVelocity(_info, _ecm);
      this->dataPtr->ApplyWindForce(_info, _ecm);
    }
  }
}

IGNITION_ADD_PLUGIN(WindEffects, System,
  WindEffects::ISystemConfigure,
  WindEffects::ISystemPreUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(WindEffects, "ignition::gazebo::systems::WindEffects")
