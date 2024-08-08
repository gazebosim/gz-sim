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

#include "WindEffects.hh"

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <google/protobuf/message.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/msgs/wind.pb.h>

#include <string>
#include <utility>
#include <vector>

#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/AdditivelySeparableScalarField3.hh>
#include <gz/math/PiecewiseScalarField3.hh>
#include <gz/math/Polynomial3.hh>
#include <gz/math/Region3.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Vector4.hh>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/sensors/Noise.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/SdfEntityCreator.hh"

#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocitySeed.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/WindMode.hh"

#include "gz/sim/Link.hh"

using namespace gz;
using namespace sim;
using namespace systems;

namespace {
  using ScalingFactor =
      math::AdditivelySeparableScalarField3d<math::Polynomial3d>;
  using PiecewiseScalingFactor = math::PiecewiseScalarField3d<ScalingFactor>;

  //////////////////////////////////////////////////
  ScalingFactor MakeConstantScalingFactor(double _value)
  {
    return ScalingFactor(
        _value / 3.,
        math::Polynomial3d::Constant(_value),
        math::Polynomial3d::Constant(_value),
        math::Polynomial3d::Constant(_value));
  }

  //////////////////////////////////////////////////
  math::Polynomial3d LoadPolynomial3d(const sdf::ElementPtr &_sdf)
  {
    math::Vector4<double> coeffs;
    std::istringstream(_sdf->Get<std::string>()) >> coeffs;
    return math::Polynomial3d(std::move(coeffs));
  }

  //////////////////////////////////////////////////
  ScalingFactor LoadScalingFactor(const sdf::ElementPtr &_sdf)
  {
    if (!_sdf->GetFirstElement())
    {
      return MakeConstantScalingFactor(_sdf->Get<double>());
    }
    double k = 1.;
    if (_sdf->HasElement("k"))
    {
      k = _sdf->GetElementImpl("k")->Get<double>();
    }
    math::Polynomial3d p = math::Polynomial3d::Constant(0.);
    if (_sdf->HasElement("px"))
    {
      p = LoadPolynomial3d(_sdf->GetElementImpl("px"));
    }
    math::Polynomial3d q = math::Polynomial3d::Constant(0.);
    if (_sdf->HasElement("qy"))
    {
      q = LoadPolynomial3d(_sdf->GetElementImpl("qy"));
    }
    math::Polynomial3d r = math::Polynomial3d::Constant(0.);
    if (_sdf->HasElement("rz"))
    {
      r = LoadPolynomial3d(_sdf->GetElementImpl("rz"));
    }
    return ScalingFactor(k, std::move(p), std::move(q), std::move(r));
  }

  //////////////////////////////////////////////////
  math::Intervald
  LoadIntervald(const sdf::ElementPtr _sdf, const std::string &_prefix)
  {
    bool leftClosed = false;
    double leftValue = -math::INF_D;
    const std::string geAttrName = _prefix + "ge";
    const std::string gtAttrName = _prefix + "gt";
    if (_sdf->HasAttribute(geAttrName) && _sdf->HasAttribute(gtAttrName))
    {
      gzerr << "Attributes '" << geAttrName << "' and '" << gtAttrName << "'"
             << " are mutually exclusive. Ignoring both." << std::endl;
    }
    else if (_sdf->HasAttribute(geAttrName))
    {
      sdf::ParamPtr sdfGeAttrValue = _sdf->GetAttribute(geAttrName);
      if (!sdfGeAttrValue->Get<double>(leftValue))
      {
        gzerr << "Invalid '" << geAttrName << "' attribute value. "
               << "Ignoring." << std::endl;
      }
      else
      {
        leftClosed = true;
      }
    }
    else if (_sdf->HasAttribute(gtAttrName))
    {
      sdf::ParamPtr sdfGtAttrValue = _sdf->GetAttribute(gtAttrName);
      if(!sdfGtAttrValue->Get<double>(leftValue))
      {
        gzerr << "Invalid '" << gtAttrName << "' attribute value. "
               << "Ignoring." << std::endl;
      }
    }

    bool rightClosed = false;
    double rightValue = math::INF_D;
    const std::string leAttrName = _prefix + "le";
    const std::string ltAttrName = _prefix + "lt";
    if (_sdf->HasAttribute(leAttrName) && _sdf->HasAttribute(ltAttrName))
    {
      gzerr << "Attributes '" << leAttrName << "' and '" << ltAttrName << "'"
             << " are mutually exclusive. Ignoring both." << std::endl;
    }
    else if (_sdf->HasAttribute(leAttrName))
    {
      sdf::ParamPtr sdfLeAttrValue = _sdf->GetAttribute(leAttrName);
      if (!sdfLeAttrValue->Get<double>(rightValue))
      {
        gzerr << "Invalid '" << leAttrName << "' attribute value. "
               << "Ignoring." << std::endl;
      }
      else
      {
        rightClosed = true;
      }
    }
    else if (_sdf->HasAttribute(ltAttrName))
    {
      sdf::ParamPtr sdfLtAttrValue = _sdf->GetAttribute(ltAttrName);
      if (!sdfLtAttrValue->Get<double>(rightValue))
      {
        gzerr << "Invalid '" << gtAttrName << "'"
               << "attribute value. Ignoring."
               << std::endl;
      }
    }

    return math::Intervald(leftValue, leftClosed, rightValue, rightClosed);
  }

  //////////////////////////////////////////////////
  PiecewiseScalingFactor LoadPiecewiseScalingFactor(const sdf::ElementPtr _sdf)
  {
    if (!_sdf->HasElement("when"))
    {
      return PiecewiseScalingFactor::Throughout(LoadScalingFactor(_sdf));
    }
    std::vector<PiecewiseScalingFactor::Piece> pieces;
    for (sdf::ElementPtr sdfPiece = _sdf->GetElement("when");
         sdfPiece; sdfPiece = sdfPiece->GetNextElement("when"))
    {
      pieces.push_back({
          math::Region3d(LoadIntervald(sdfPiece, "x"),
                         LoadIntervald(sdfPiece, "y"),
                         LoadIntervald(sdfPiece, "z")),
          LoadScalingFactor(sdfPiece),
      });
    }
    return PiecewiseScalingFactor(std::move(pieces));
  }
}  // namespace

/// \brief Private WindEffects data class.
class gz::sim::systems::WindEffectsPrivate
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
  public: PiecewiseScalingFactor forceApproximationScalingFactor;

  /// \brief Noise added to magnitude.
  public: sensors::NoisePtr noiseMagnitude;

  /// \brief Noise added to direction.
  public: sensors::NoisePtr noiseDirection;

  /// \brief Noise added to Z axis.
  public: sensors::NoisePtr noiseVertical;

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Set during Load to true if the configuration for the plugin is
  /// valid.
  public: bool validConfig{false};

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
          gzerr << "Please set <horizontal><magnitude><time_for_rise> to a "
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
          gzerr << "Please set <horizontal><direction><time_for_rise> to a "
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
        gzerr << "Please set <horizontal><magnitude><time_for_rise> to a "
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
    this->forceApproximationScalingFactor =
        LoadPiecewiseScalingFactor(sdfForceApprox);
  }
  else
  {
    this->forceApproximationScalingFactor =
        PiecewiseScalingFactor::Throughout(MakeConstantScalingFactor(1.));
  }

  // It doesn't make sense to be negative, that would be negative wind drag.
  if (this->forceApproximationScalingFactor.Minimum() < 0.)
  {
    gzerr << "<force_approximation_scaling_factor> must "
           << "always be a nonnegative quantity" << std::endl;
    return;
  }

  this->validConfig = true;
}

//////////////////////////////////////////////////
void WindEffectsPrivate::SetupTransport(const std::string &_worldName)
{
  auto validWorldName = transport::TopicUtils::AsValidTopic(_worldName);
  if (validWorldName.empty())
  {
    gzerr << "Failed to setup transport, invalid world name [" << _worldName
           << "]" << std::endl;
    return;
  }

  // Wind seed velocity topic
  this->node.Subscribe("/world/" + validWorldName + "/wind",
                       &WindEffectsPrivate::OnWindMsg, this);

  // Wind info service
  this->node.Advertise("/world/" + validWorldName + "/wind_info",
                       &WindEffectsPrivate::WindInfoService, this);
}

//////////////////////////////////////////////////
void WindEffectsPrivate::UpdateWindVelocity(const UpdateInfo &_info,
                                            EntityComponentManager &_ecm)
{
  GZ_PROFILE("WindEffectsPrivate::UpdateWindVelocity");
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
               std::sin(2 * GZ_PI * simTime / this->magnitudeSinPeriod);

  if (this->noiseMagnitude)
  {
    magnitude = this->noiseMagnitude->Apply(magnitude);
  }

  // Compute horizontal direction
  double direction =
      GZ_RTOD(atan2(windLinVelSeed->Data().Y(), windLinVelSeed->Data().X()));

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
               std::sin(2 * GZ_PI * simTime / this->orientationSinPeriod);

  if (this->noiseDirection)
    direction = this->noiseDirection->Apply(direction);

  // Apply wind velocity
  math::Vector3d windVel;
  windVel.X(magnitude * std::cos(GZ_DTOR(direction)));
  windVel.Y(magnitude * std::sin(GZ_DTOR(direction)));

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
  GZ_PROFILE("WindEffectsPrivate::ApplyWindForce");
  auto windVel =
      _ecm.Component<components::WorldLinearVelocity>(this->windEntity);
  if (!windVel)
    return;

  Link link;

  _ecm.Each<components::Link,
            components::Inertial,
            components::WindMode,
            components::WorldPose,
            components::WorldLinearVelocity>(
      [&](const Entity &_entity,
          components::Link *,
          components::Inertial *_inertial,
          components::WindMode *_windMode,
          components::WorldPose *_linkPose,
          components::WorldLinearVelocity *_linkVel) -> bool
      {
        // Skip links for which the wind is disabled
        if (!_windMode->Data())
        {
          return true;
        }

        link.ResetEntity(_entity);

        double forceScalingFactor =
            this->forceApproximationScalingFactor(_linkPose->Data().Pos());
        if (std::isnan(forceScalingFactor))
        {
          forceScalingFactor = 0.;
        }

        const math::Vector3d windForce =
            _inertial->Data().MassMatrix().Mass() *
            forceScalingFactor * (windVel->Data() - _linkVel->Data());

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
  GZ_PROFILE("WindEffects::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Process commands
  this->dataPtr->ProcessCommandQueue(_ecm);

  if (!this->dataPtr->validConfig)
    return;

  _ecm.EachNew<components::Link, components::WindMode>(
      [&](const Entity &_entity, components::Link *,
          components::WindMode *_windMode) -> bool
  {
    if (_windMode->Data())
    {
      Link link(_entity);
      link.EnableVelocityChecks(_ecm, true);
    }
    return true;
  });

  if (_info.paused)
    return;

  if (!this->dataPtr->currentWindInfo.enable_wind())
    return;

  this->dataPtr->UpdateWindVelocity(_info, _ecm);
  this->dataPtr->ApplyWindForce(_info, _ecm);

}

GZ_ADD_PLUGIN(WindEffects, System,
  WindEffects::ISystemConfigure,
  WindEffects::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(WindEffects, "gz::sim::systems::WindEffects")
