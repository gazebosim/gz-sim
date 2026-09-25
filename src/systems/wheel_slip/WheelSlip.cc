/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "WheelSlip.hh"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/Event.hh>
#include <gz/common/Profiler.hh>
#include <gz/physics/ContactProperties.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/SlipComplianceCmd.hh"
#include "gz/sim/components/WheelSlipCmd.hh"
#include "gz/sim/physics/Events.hh"

using namespace gz;
using namespace sim;
using namespace systems;

// Adapted from osrf/Gazebo WheelSlipPlugin
// https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2950/
class gz::sim::systems::WheelSlipPrivate
{
  /// \brief Initialize the plugin
  public: bool Load(EntityComponentManager &_ecm, sdf::ElementPtr _sdf);

  /// \brief Update wheel slip plugin data based on physics data
  /// \param[in] _ecm Mutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  /// \brief Update surface parameters from the transport parameter registry.
  public: void UpdateParams();

  public: using P = physics::FeaturePolicy3d;
  public: using F = physics::SetContactPropertiesCallbackFeature;
  /// \brief The callback for CollectContactSurfaceProperties.
  /// This is where we set surface properties.
  /// \param[in] _collision1 The first colliding body.
  /// \param[in] _collision2 The second colliding body.
  /// \param[in,out] _params The contact surface parameters to be set by this
  /// system.
  public: void SetSurfaceProperties(
    const Entity &_collision1,
    const Entity &_collision2,
    F::ContactSurfaceParams<P> &_params);

  /// \brief Model interface
  public: Model model{kNullEntity};

  public: class LinkSurfaceParams
    {
      /// \brief Pointer to wheel spin joint.
      public: Entity joint;

      /// \brief Pointer to wheel spin collision entity.
      public: Entity collision;

      /// \brief Unitless wheel slip compliance in lateral direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLateral = 0;

      /// \brief Unitless wheel slip compliance in longitudinal direction.
      /// The parameter should be non-negative,
      /// with a value of zero allowing no slip
      /// and larger values allowing increasing slip.
      public: double slipComplianceLongitudinal = 0;

      /// \brief Wheel normal force estimate used to compute slip
      /// compliance, which takes units of 1/N.
      public: double wheelNormalForce = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheelRadius = 0;

      /// \brief Wheel collision primary friction coefficient.
      public: double frictionCoeffPrimary = 1.0;

      /// \brief Wheel collision secondary friction coefficient.
      public: double frictionCoeffSecondary = 1.0;
    };

  /// \brief The map relating links to their respective surface parameters.
  public: std::map<Entity, LinkSurfaceParams> mapLinkSurfaceParams;

  /// \brief Vector2d equality comparison function.
  public: std::function<bool(const std::vector<double> &,
              const std::vector<double> &)>
          vecEql { [](const std::vector<double> &_a,
              const std::vector<double> &_b)
                    {
                      if (_a.size() != _b.size() ||
                          _a.size() < 2 ||_b.size() < 2)
                      {
                        return false;
                      }

                      for (size_t i = 0; i < _a.size(); i++)
                      {
                        if (std::abs(_a[i] - _b[i]) > 1e-6)
                          return false;
                      }
                      return true;
                    }};

  public: bool validConfig{false};
  public: bool initialized{false};

  /// \brief Transport parameter registry. A number of surface params will
  /// be exposed in the registry for user configuration.
  public: transport::parameters::ParametersRegistry *registry{nullptr};

  /// \brief The map relating links to their parameters in the transport
  /// parameter registry. The elements are:
  /// <entity_id, <param_name, param_value>>.
  public: std::unordered_map<Entity,
      std::unordered_map<std::string, std::string>> mapLinkParamNames;

  /// \brief Lateral slip compliance parameter name in the parameter registry.
  public: static constexpr std::string_view kSlipComplianceLateralParamName =
      "slip_compliance_lateral";

  /// \brief Longitudinal slip compliance parameter name in the parameter
  /// registry.
  public: static constexpr std::string_view
      kSlipComplianceLongitudinalParamName = "slip_compliance_longitudinal";

  /// \brief Primary friction coefficient parameter name in the parameter
  /// registry.
  public: static constexpr std::string_view
      kFrictionCoefficientPrimaryParamName = "friction_coefficient_primary";

  /// \brief Secondary friction coefficient parameter name in the parameter
  /// registry.
  public: static constexpr std::string_view
      kFrictionCoefficientSecondaryParamName = "friction_coefficient_secondary";

  /// \brief Event manager.
  public: EventManager* eventManager{nullptr};

  /// \brief Connection to CollectContactSurfaceProperties event.
  public: common::ConnectionPtr eventConnection;
};

/////////////////////////////////////////////////
bool WheelSlipPrivate::Load(EntityComponentManager &_ecm,
                            sdf::ElementPtr _sdf)
{
  const std::string modelName = this->model.Name(_ecm);

  if (!_sdf->HasElement("wheel"))
  {
    gzerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return false;
  }

  // Read each wheel element
  for (auto wheelElem = _sdf->GetElement("wheel"); wheelElem;
      wheelElem = wheelElem->GetNextElement("wheel"))
  {
    if (!wheelElem->HasAttribute("link_name"))
    {
      gzerr << "wheel element missing link_name attribute" << std::endl;
      continue;
    }

    // Get link name
    auto linkName = wheelElem->Get<std::string>("link_name");

    LinkSurfaceParams params;
    if (wheelElem->HasElement("slip_compliance_lateral"))
    {
      params.slipComplianceLateral =
        wheelElem->Get<double>("slip_compliance_lateral");
    }
    if (wheelElem->HasElement("slip_compliance_longitudinal"))
    {
      params.slipComplianceLongitudinal =
        wheelElem->Get<double>("slip_compliance_longitudinal");
    }
    if (wheelElem->HasElement("wheel_normal_force"))
    {
      params.wheelNormalForce = wheelElem->Get<double>("wheel_normal_force");
    }

    if (wheelElem->HasElement("wheel_radius"))
    {
      params.wheelRadius = wheelElem->Get<double>("wheel_radius");
    }

    auto link = Link(this->model.LinkByName(_ecm, linkName));
    if (!link.Valid(_ecm))
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model [" << modelName << "]"
            << std::endl;
      continue;
    }

    auto collisions =
        _ecm.ChildrenByComponents(link.Entity(), components::Collision());
    if (collisions.empty() || collisions.size() != 1)
    {
      gzerr << "There should be 1 collision in link named [" << linkName
            << "] in model [" << modelName << "]"
            << ", but " << collisions.size() << " were found"
            << std::endl;
      continue;
    }

    auto collision = collisions.front();
    params.collision = collision;
    _ecm.SetComponentData<components::EnableContactSurfaceCustomization>(
        collision, true);

    auto joints =
        _ecm.ChildrenByComponents(model.Entity(), components::Joint(),
                                  components::ChildLinkName(linkName));
    if (joints.empty() || joints.size() != 1)
    {
      gzerr << "There should be 1 parent joint for link named [" << linkName
            << "] in model [" << modelName << "]"
            << ", but " << joints.size() << " were found"
            << std::endl;
      continue;
    }

    params.joint = joints.front();

    if (params.wheelRadius <= 0)
    {
      gzerr << "Found wheel radius [" << params.wheelRadius
            << "], which is not positive"
            << " in link named [" << linkName
            << "] in model [" << modelName << "]"
            << std::endl;
      continue;
    }

    if (params.wheelNormalForce <= 0)
    {
      gzerr << "Found wheel normal force [" << params.wheelNormalForce
            << "], which is not positive"
            << " in link named [" << linkName
            << "] in model [" << modelName << "]"
            << std::endl;
      continue;
    }

    this->mapLinkSurfaceParams[link.Entity()] = params;
  }

  if (this->mapLinkSurfaceParams.empty())
  {
    gzerr << "No links and surfaces found, plugin is disabled"
           << std::endl;
    return false;
  }

  return true;
}


/////////////////////////////////////////////////
void WheelSlipPrivate::UpdateParams()
{
  if (!this->registry)
    return;

  for (auto &linkSurface : this->mapLinkSurfaceParams)
  {
    // Slip compliance lateral
    std::string paramName = this->mapLinkParamNames[
        linkSurface.first][std::string(kSlipComplianceLateralParamName)];
    auto value = std::make_unique<gz::msgs::Double>();
    auto result = this->registry->Parameter(paramName, *value);
    if (result)
    {
      if (!math::equal(linkSurface.second.slipComplianceLateral,
          value->data(), 1e-6))
      {
        gzdbg << "Parameter " << paramName << " updated from "
              << linkSurface.second.slipComplianceLateral << " to "
              << value->data() << std::endl;
        linkSurface.second.slipComplianceLateral = value->data();
      }
    }
    else
    {
      gzerr << "Failed to get parameter [" << paramName << "] :"
            << result << std::endl;
    }

    // Slip compliance longitudinal
    paramName = this->mapLinkParamNames[
        linkSurface.first][std::string(kSlipComplianceLongitudinalParamName)];
    value = std::make_unique<gz::msgs::Double>();
    result = this->registry->Parameter(paramName, *value);
    if (result)
    {
      if (!math::equal(linkSurface.second.slipComplianceLongitudinal,
          value->data(), 1e-6))
      {
        gzdbg << "Parameter " << paramName << " updated from "
              << linkSurface.second.slipComplianceLongitudinal << " to "
              << value->data() << std::endl;
        linkSurface.second.slipComplianceLongitudinal = value->data();
      }
    }
    else
    {
      gzerr << "Failed to get parameter [" << paramName << "] :"
            << result << std::endl;
    }

    // Primary friction coeff
    paramName = this->mapLinkParamNames[
        linkSurface.first][std::string(kFrictionCoefficientPrimaryParamName)];
    value = std::make_unique<gz::msgs::Double>();
    result = this->registry->Parameter(paramName, *value);
    if (result)
    {
      if (!math::equal(linkSurface.second.frictionCoeffPrimary,
          value->data(), 1e-6))
      {
        gzdbg << "Parameter " << paramName << " updated from "
              << linkSurface.second.frictionCoeffPrimary << " to "
              << value->data() << std::endl;
        linkSurface.second.frictionCoeffPrimary = value->data();
      }
    }
    else
    {
      gzerr << "Failed to get parameter [" << paramName << "] :"
            << result << std::endl;
    }

    // Secondary friction coeff
    paramName = this->mapLinkParamNames[
        linkSurface.first][std::string(kFrictionCoefficientSecondaryParamName)];
    value = std::make_unique<gz::msgs::Double>();
    result = this->registry->Parameter(paramName, *value);
    if (result)
    {
      if (!math::equal(linkSurface.second.frictionCoeffSecondary,
          value->data(), 1e-6))
      {
        gzdbg << "Parameter " << paramName << " updated from "
              << linkSurface.second.frictionCoeffSecondary << " to "
              << value->data() << std::endl;
        linkSurface.second.frictionCoeffSecondary = value->data();
      }
    }
    else
    {
      gzerr << "Failed to get parameter [" << paramName << "] :"
            << result << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void WheelSlipPrivate::Update(EntityComponentManager &_ecm)
{
  this->UpdateParams();

  for (auto &linkSurface : this->mapLinkSurfaceParams)
  {
    auto &params = linkSurface.second;
    const auto * wheelSlipCmdComp =
      _ecm.Component<components::WheelSlipCmd>(linkSurface.first);
    if (wheelSlipCmdComp)
    {
      const auto & wheelSlipCmdParams = wheelSlipCmdComp->Data();
      bool changed = (!math::equal(
          params.slipComplianceLateral,
          wheelSlipCmdParams.slip_compliance_lateral(),
          1e-6)) ||
        (!math::equal(
          params.slipComplianceLongitudinal,
          wheelSlipCmdParams.slip_compliance_longitudinal(),
          1e-6));

      if (changed)
      {
        params.slipComplianceLateral =
          wheelSlipCmdParams.slip_compliance_lateral();
        params.slipComplianceLongitudinal =
          wheelSlipCmdParams.slip_compliance_longitudinal();
      }
      _ecm.RemoveComponent<components::WheelSlipCmd>(linkSurface.first);
    }

    // get user-defined normal force constant
    double force = params.wheelNormalForce;

    auto joint = params.joint;

    auto spinAngularVelocityComp =
        _ecm.Component<components::JointVelocity>(joint);

    if (!spinAngularVelocityComp || spinAngularVelocityComp->Data().empty())
      continue;
    double spinAngularVelocity = spinAngularVelocityComp->Data()[0];

    // As discussed in WheelSlip.hh, the slip1 and slip2
    // parameters have units of inverse viscous damping:
    // [linear velocity / force] or [m / s / N].
    // Since the slip compliance parameters supplied to the plugin
    // are unitless, they must be scaled by a linear speed and force
    // magnitude.
    // The force is taken from a user-defined constant that should roughly
    // match the steady-state normal force at the wheel.
    // The linear speed is computed dynamically at each time step as
    // radius * spin angular velocity.
    // This choice of linear speed corresponds to the denominator of
    // the slip ratio during acceleration (see equation (1) in
    // Yoshida, Hamano 2002 DOI 10.1109/ROBOT.2002.1013712
    // "Motion dynamics of a rover with slip-based traction model").
    // The acceleration form is more well-behaved numerically at low-speed
    // and when the vehicle is at rest than the braking form,
    // so it is used for both slip directions.
    double speed = params.wheelRadius * std::abs(spinAngularVelocity);
    double slip1 = speed / force * params.slipComplianceLateral;
    double slip2 = speed / force * params.slipComplianceLongitudinal;

    components::SlipComplianceCmd newSlipCmdComp({slip1, slip2});

    auto currSlipCmdComp =
        _ecm.Component<components::SlipComplianceCmd>(params.collision);
    if (currSlipCmdComp)
    {
      *currSlipCmdComp = newSlipCmdComp;
      _ecm.SetChanged(params.collision, components::SlipComplianceCmd::typeId,
                      ComponentState::PeriodicChange);
    }
    else
    {
      _ecm.CreateComponent(params.collision, newSlipCmdComp);
    }
  }
}

//////////////////////////////////////////////////
WheelSlip::WheelSlip()
  : dataPtr(std::make_unique<WheelSlipPrivate>())
{
}

//////////////////////////////////////////////////
void WheelSlip::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "WheelSlip plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();
  this->dataPtr->validConfig = this->dataPtr->Load(_ecm, sdfClone);

  using P = physics::FeaturePolicy3d;
  using F = physics::SetContactPropertiesCallbackFeature;
  this->dataPtr->eventManager = &_eventMgr;
  this->dataPtr->eventConnection = this->dataPtr->eventManager->
    Connect<events::CollectContactSurfaceProperties>(
    [this](
      const Entity &_collision1,
      const Entity &_collision2,
      const math::Vector3d &/*_point*/,
      const std::optional<math::Vector3d> /* _force */,
      const std::optional<math::Vector3d> /*_normal*/,
      const std::optional<double> /*_depth */,
      const size_t /*_numContactsOnCollision*/,
      F::ContactSurfaceParams<P>& _params)
    {
      this->dataPtr->SetSurfaceProperties(_collision1, _collision2, _params);
    }
  );
}

//////////////////////////////////////////////////
void WheelSlip::ConfigureParameters(
    gz::transport::parameters::ParametersRegistry &_registry,
    gz::sim::EntityComponentManager &_ecm)
{
  this->dataPtr->registry = &_registry;

  for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
  {
    auto linkEnt = linkSurface.first;
    std::string prefix = std::string("WheelSlip.") +
                         sim::scopedName(linkEnt, _ecm, ".", false);
    std::string paramName = prefix + "." +
        std::string(WheelSlipPrivate::kSlipComplianceLateralParamName);
    auto value = std::make_unique<gz::msgs::Double>();
    value->set_data(linkSurface.second.slipComplianceLateral);
    auto result = this->dataPtr->registry->DeclareParameter(
        paramName, std::move(value));
    if (!result)
    {
      gzerr << "Unable to declare parameter " << paramName << std::endl;
    }
    this->dataPtr->mapLinkParamNames[linkEnt][
        std::string(WheelSlipPrivate::kSlipComplianceLateralParamName)] =
        paramName;

    paramName = prefix + "." +
        std::string(WheelSlipPrivate::kSlipComplianceLongitudinalParamName);
    value = std::make_unique<gz::msgs::Double>();
    value->set_data(linkSurface.second.slipComplianceLongitudinal);
    result = this->dataPtr->registry->DeclareParameter(
        paramName, std::move(value));
    if (!result)
    {
      gzerr << "Unable to declare parameter " << paramName << std::endl;
    }
    this->dataPtr->mapLinkParamNames[linkEnt][
        std::string(WheelSlipPrivate::kSlipComplianceLongitudinalParamName)] =
        paramName;

    paramName = prefix + "." +
        std::string(WheelSlipPrivate::kFrictionCoefficientPrimaryParamName);
    value = std::make_unique<gz::msgs::Double>();
    value->set_data(linkSurface.second.frictionCoeffPrimary);
    result = this->dataPtr->registry->DeclareParameter(
        paramName, std::move(value));
    if (!result)
    {
      gzerr << "Unable to declare parameter " << paramName << std::endl;
    }
    this->dataPtr->mapLinkParamNames[linkEnt][
        std::string(WheelSlipPrivate::kFrictionCoefficientPrimaryParamName)] =
        paramName;

    paramName = prefix + "." +
        std::string(WheelSlipPrivate::kFrictionCoefficientSecondaryParamName);
    value = std::make_unique<gz::msgs::Double>();
    value->set_data(linkSurface.second.frictionCoeffSecondary);
    result = this->dataPtr->registry->DeclareParameter(
        paramName, std::move(value));
    if (!result)
    {
      gzerr << "Unable to declare parameter " << paramName << std::endl;
    }
    this->dataPtr->mapLinkParamNames[linkEnt][
        std::string(WheelSlipPrivate::kFrictionCoefficientSecondaryParamName)] =
        paramName;
  }
}

//////////////////////////////////////////////////
void WheelSlip::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("WheelSlip::PreUpdate");

  if (!this->dataPtr->validConfig)
    return;

  if (!this->dataPtr->initialized)
  {
    if (this->dataPtr->validConfig)
    {
      for (const auto &linkSurface : this->dataPtr->mapLinkSurfaceParams)
      {
        if (!_ecm.Component<components::WorldAngularVelocity>(
                linkSurface.first))
        {
          _ecm.CreateComponent(linkSurface.first, components::JointVelocity());
        }
        if (!_ecm.Component<components::JointVelocity>(
                linkSurface.second.joint))
        {
          _ecm.CreateComponent(linkSurface.second.joint,
                               components::JointVelocity());
        }
      }
    }
    this->dataPtr->initialized = true;
  }
  else
  {
    // Nothing left to do if paused.
    if (_info.paused)
      return;

    this->dataPtr->Update(_ecm);
  }
}

//////////////////////////////////////////////////
void WheelSlipPrivate::SetSurfaceProperties(
  const Entity &_collision1,
  const Entity &_collision2,
  F::ContactSurfaceParams<P> &_params)
{
  // Sets surface friction properties
  // \todo(iche033) Consider setting slip compliance here in this
  // callback function instead of using SlipComplianceCmd components.
  for (const auto &linkSurface : this->mapLinkSurfaceParams)
  {
    if (linkSurface.second.collision == _collision1 ||
        linkSurface.second.collision == _collision2)
    {
      _params.frictionCoeff = linkSurface.second.frictionCoeffPrimary;
      _params.secondaryFrictionCoeff =
          linkSurface.second.frictionCoeffSecondary;
      break;
    }
  }
}

GZ_ADD_PLUGIN(WheelSlip,
                    System,
                    WheelSlip::ISystemConfigure,
                    WheelSlip::ISystemConfigureParameters,
                    WheelSlip::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WheelSlip,
                          "gz::sim::systems::WheelSlip")
