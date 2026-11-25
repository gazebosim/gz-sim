/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "HydrostaticPressure.hh"

#include <gz/msgs/fluid_pressure.pb.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Profiler.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private HydrostaticPressure data class.
class gz::sim::systems::HydrostaticPressurePrivate
{
  /// \brief Convert a depth in meters to hydrostatic pressure in Pascals
  /// using the Fofonoff & Millard (1983) polynomial.
  ///
  /// The polynomial is published as p -> z; a Newton iteration from a
  /// linear seed inverts it to z -> p. Three iterations converge to
  /// sub-mPa accuracy at 1000 m. The constant g = 9.80665 m/s^2
  /// corresponds to ~45 deg latitude and contributes <= 2.6 m of error
  /// at 1000 m due to latitude variation.
  /// \param[in] _depthM Depth below the fluid surface in meters.
  /// \return Hydrostatic pressure (above surface) in Pa.
  public: double DepthToPressurePa(double _depthM) const;

  /// \brief Model the plugin is attached to.
  public: Model model{kNullEntity};

  /// \brief Link entity whose Z position drives the pressure reading.
  public: Entity linkEntity{kNullEntity};

  /// \brief Topic used to publish FluidPressure messages.
  public: std::string topic;

  /// \brief Value placed in the message header frame_id.
  public: std::string frameId;

  /// \brief Fluid density in kg/m^3. Defaults to seawater.
  public: double fluidDensity{1025.0};

  /// \brief Pressure at the fluid surface in Pa.
  public: double surfacePressurePa{101325.0};

  /// \brief Magnitude of gravity in m/s^2 used by the F&M polynomial.
  public: double gravity{9.80665};

  /// \brief World Z of the fluid/air interface in meters.
  public: double surfaceZ{0.0};

  /// \brief Standard deviation of additive Gaussian noise on the
  /// reading in Pa.
  public: double noiseStddev{0.0};

  /// \brief Publishing period derived from the update rate.
  public: std::chrono::steady_clock::duration updatePeriod{
      std::chrono::milliseconds(50)};

  /// \brief Sim time of the last published sample.
  public: std::chrono::steady_clock::duration lastPubSimTime{0};

  /// \brief Whether the next PostUpdate should publish unconditionally.
  public: bool firstPublishPending{true};

  /// \brief Transport node used to advertise the topic.
  public: transport::Node node;

  /// \brief Publisher for FluidPressure messages.
  public: transport::Node::Publisher pub;

  /// \brief Cached message reused on every publish to avoid
  /// per-iteration allocations.
  public: msgs::FluidPressure msg;

  /// \brief True once Configure has fully succeeded.
  public: bool initialized{false};
};

//////////////////////////////////////////////////
double HydrostaticPressurePrivate::DepthToPressurePa(double _depthM) const
{
  constexpr double c1 = 9.72659;
  constexpr double c2 = -2.2512e-5;
  constexpr double c3 = 2.279e-10;
  constexpr double c4 = -1.82e-15;
  const double rhs = this->gravity * _depthM;
  // First-order seed.
  double p = rhs / c1;
  for (int i = 0; i < 3; ++i)
  {
    const double poly = (((c4 * p + c3) * p + c2) * p + c1) * p;
    const double dpoly =
        ((4.0 * c4 * p + 3.0 * c3) * p + 2.0 * c2) * p + c1;
    p -= (poly - rhs) / dpoly;
  }
  // Convert decibars to Pascals.
  return p * 1e4;
}

//////////////////////////////////////////////////
HydrostaticPressure::HydrostaticPressure()
  : dataPtr(std::make_unique<HydrostaticPressurePrivate>())
{
}

//////////////////////////////////////////////////
HydrostaticPressure::~HydrostaticPressure() = default;

//////////////////////////////////////////////////
void HydrostaticPressure::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "HydrostaticPressure plugin must be attached to a model entity. "
          << "Entity [" << _entity << "] is not a model." << std::endl;
    return;
  }

  const std::string linkName =
      _sdf->Get<std::string>("link_name", "base_link").first;
  this->dataPtr->topic = _sdf->Get<std::string>("topic", "").first;
  this->dataPtr->frameId = _sdf->Get<std::string>("frame_id", linkName).first;
  this->dataPtr->fluidDensity =
      _sdf->Get<double>("fluid_density", this->dataPtr->fluidDensity).first;
  this->dataPtr->surfacePressurePa = _sdf->Get<double>(
      "surface_pressure_pa", this->dataPtr->surfacePressurePa).first;
  this->dataPtr->gravity =
      _sdf->Get<double>("gravity", this->dataPtr->gravity).first;
  this->dataPtr->surfaceZ =
      _sdf->Get<double>("surface_z", this->dataPtr->surfaceZ).first;
  this->dataPtr->noiseStddev =
      _sdf->Get<double>("noise_stddev", this->dataPtr->noiseStddev).first;

  const double rateHz = _sdf->Get<double>("update_rate", 20.0).first;
  if (rateHz <= 0.0)
  {
    gzerr << "HydrostaticPressure: update_rate must be > 0, got [" << rateHz
          << "]; using 20 Hz." << std::endl;
    this->dataPtr->updatePeriod = std::chrono::milliseconds(50);
  }
  else
  {
    this->dataPtr->updatePeriod = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / rateHz));
  }

  // Default topic: /<model_name>/pressure. Lets the plugin be dropped into
  // any model SDF without explicit topic configuration.
  if (this->dataPtr->topic.empty())
  {
    this->dataPtr->topic =
        "/" + this->dataPtr->model.Name(_ecm) + "/pressure";
  }

  this->dataPtr->linkEntity =
      this->dataPtr->model.LinkByName(_ecm, linkName);
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    gzerr << "HydrostaticPressure: link [" << linkName
          << "] not found in model [" << this->dataPtr->model.Name(_ecm)
          << "]; sensor disabled." << std::endl;
    return;
  }

  this->dataPtr->pub = this->dataPtr->node.Advertise<msgs::FluidPressure>(
      this->dataPtr->topic);
  if (!this->dataPtr->pub)
  {
    gzerr << "HydrostaticPressure: failed to advertise topic ["
          << this->dataPtr->topic << "]; sensor disabled." << std::endl;
    return;
  }

  // Build static message fields once to avoid per-publish allocations.
  this->dataPtr->msg.Clear();
  auto *stamp = this->dataPtr->msg.mutable_header()->mutable_stamp();
  stamp->set_sec(0);
  stamp->set_nsec(0);
  auto *frame = this->dataPtr->msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->dataPtr->frameId);
  this->dataPtr->msg.set_variance(
      this->dataPtr->noiseStddev * this->dataPtr->noiseStddev);

  this->dataPtr->initialized = true;
  gzmsg << "HydrostaticPressure: publishing on [" << this->dataPtr->topic
        << "] at " << rateHz << " Hz, link=[" << linkName
        << "], fluid_density=" << this->dataPtr->fluidDensity
        << " kg/m^3, surface_pressure=" << this->dataPtr->surfacePressurePa
        << " Pa, surface_z=" << this->dataPtr->surfaceZ << " m"
        << std::endl;
}

//////////////////////////////////////////////////
void HydrostaticPressure::PostUpdate(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("HydrostaticPressure::PostUpdate");

  if (!this->dataPtr->initialized || _info.paused)
    return;

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // Throttle to the configured rate using sim time so the cadence is
  // independent of the real-time factor.
  if (!this->dataPtr->firstPublishPending)
  {
    if (_info.simTime - this->dataPtr->lastPubSimTime <
        this->dataPtr->updatePeriod)
    {
      return;
    }
  }
  this->dataPtr->lastPubSimTime = _info.simTime;
  this->dataPtr->firstPublishPending = false;

  // World pose of the tracked link. worldPose() walks the parent chain
  // and accounts for joint offsets so we do not need to maintain a
  // WorldPose component ourselves.
  const math::Pose3d pose = worldPose(this->dataPtr->linkEntity, _ecm);

  // Above water (linkZ > surfaceZ) we report surface pressure rather
  // than a negative depth - a real depth sensor floors at atmospheric.
  const double depthM =
      std::max(0.0, this->dataPtr->surfaceZ - pose.Z());
  double pressurePa = this->dataPtr->surfacePressurePa +
      this->dataPtr->DepthToPressurePa(depthM);

  if (this->dataPtr->noiseStddev > 0.0)
  {
    pressurePa += math::Rand::DblNormal(0.0, this->dataPtr->noiseStddev);
  }

  const auto secs =
      std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
  const auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime - secs);
  this->dataPtr->msg.mutable_header()->mutable_stamp()->set_sec(secs.count());
  this->dataPtr->msg.mutable_header()->mutable_stamp()->set_nsec(
      static_cast<int32_t>(nsec.count()));
  this->dataPtr->msg.set_pressure(pressurePa);

  this->dataPtr->pub.Publish(this->dataPtr->msg);
}

GZ_ADD_PLUGIN(HydrostaticPressure,
              System,
              HydrostaticPressure::ISystemConfigure,
              HydrostaticPressure::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(HydrostaticPressure,
                    "gz::sim::systems::HydrostaticPressure")
