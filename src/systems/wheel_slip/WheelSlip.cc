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
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/SlipComplianceCmd.hh"

#include "WheelSlip.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Adapted from osrf/Gazebo WheelSlipPlugin
// https://bitbucket.org/osrf/gazebo/pull-requests/2950
class ignition::gazebo::systems::WheelSlipPrivate
{
  public: bool Load(const EntityComponentManager &_ecm, sdf::ElementPtr _sdf);

  public: void Update(EntityComponentManager &_ecm);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Commanded joint force
  public: double jointForceCmd;

  /// \brief mutex to protect jointForceCmd
  public: std::mutex jointForceCmdMutex;

  /// \brief Model interface
  public: Model model{kNullEntity};

  public: class LinkSurfaceParams{
      /// \brief Pointer to wheel spin joint.
      public: Entity joint;

      /// \brief Pointer to ODESurfaceParams object.
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
      /// compliance for ODE, which takes units of 1/N.
      public: double wheelNormalForce = 0;

      /// \brief Wheel radius extracted from collision shape if not
      /// specified as xml parameter.
      public: double wheelRadius = 0;
    };

  /// \brief TODO
  public: std::map<Entity, LinkSurfaceParams> mapLinkSurfaceParams;

  public: bool validConfig{false};
  public: bool initialized{false};
};

/////////////////////////////////////////////////
bool WheelSlipPrivate::Load(const EntityComponentManager &_ecm,
                            sdf::ElementPtr _sdf)
{
  const std::string modelName = this->model.Name(_ecm);

  if (!_sdf->HasElement("wheel"))
  {
    ignerr << "No wheel tags specified, plugin is disabled" << std::endl;
    return false;
  }

  // Read each wheel element
  for (auto wheelElem = _sdf->GetElement("wheel"); wheelElem;
      wheelElem = wheelElem->GetNextElement("wheel"))
  {
    if (!wheelElem->HasAttribute("link_name"))
    {
      ignerr << "wheel element missing link_name attribute" << std::endl;
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
      ignerr << "Could not find link named [" << linkName
            << "] in model [" << modelName << "]"
            << std::endl;
      continue;
    }

    auto collisions =
        _ecm.ChildrenByComponents(link.Entity(), components::Collision());
    if (collisions.empty() || collisions.size() != 1)
    {
      ignerr << "There should be 1 collision in link named [" << linkName
            << "] in model [" << modelName << "]"
            << ", but " << collisions.size() << " were found"
            << std::endl;
      continue;
    }


    auto collision = collisions.front();

    params.collision = collision;

    auto joints =
        _ecm.ChildrenByComponents(model.Entity(), components::Joint(),
                                  components::ChildLinkName(linkName));
    if (joints.empty() || joints.size() != 1)
    {
      ignerr << "There should be 1 parent joint for link named [" << linkName
            << "] in model [" << modelName << "]"
            << ", but " << joints.size() << " were found"
            << std::endl;
      continue;
    }

    params.joint = joints.front();

    if (params.wheelRadius <= 0)
    {
      ignerr << "Found wheel radius [" << params.wheelRadius
            << "], which is not positive"
            << " in link named [" << linkName
            << "] in model [" << modelName << "]"
            << std::endl;
      continue;
    }

    if (params.wheelNormalForce <= 0)
    {
      ignerr << "Found wheel normal force [" << params.wheelNormalForce
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
    ignerr << "No ODE links and surfaces found, plugin is disabled"
           << std::endl;
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void WheelSlipPrivate::Update(EntityComponentManager &_ecm)
{
  for (const auto &linkSurface : this->mapLinkSurfaceParams)
  {
    const auto &params = linkSurface.second;

    // get user-defined normal force constant
    double force = params.wheelNormalForce;

    auto joint = params.joint;

    auto spinAngularVelocityComp =
        _ecm.Component<components::JointVelocity>(joint);

    if (!spinAngularVelocityComp || spinAngularVelocityComp->Data().empty())
      continue;
    double spinAngularVelocity = spinAngularVelocityComp->Data()[0];

    // As discussed in WheelSlipPlugin.hh, the ODE slip1 and slip2
    // parameters have units of inverse viscous damping:
    // [linear velocity / force] or [m / s / N].
    // Since the slip compliance parameters supplied to the plugin
    // are unitless, they must be scaled by a linear speed and force
    // magnitude before being passed to ODE.
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

    math::Vector2d slipCmd;
    slipCmd.X() = slip1;
    slipCmd.Y() = slip2;

    components::SlipComplianceCmd newSlipCmdComp(slipCmd);

    auto currSlipCmdComp =
        _ecm.Component<components::SlipComplianceCmd>(params.collision);
    if (currSlipCmdComp )
    {
      *currSlipCmdComp = newSlipCmdComp;
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
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "WheelSlip plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();
  if (this->dataPtr->Load(_ecm, sdfClone))
  {
    this->dataPtr->validConfig = true;
  }
}

//////////////////////////////////////////////////
void WheelSlip::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  IGN_PROFILE("WheelSlip::PreUpdate");

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

IGNITION_ADD_PLUGIN(WheelSlip,
                    ignition::gazebo::System,
                    WheelSlip::ISystemConfigure,
                    WheelSlip::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(WheelSlip,
                          "ignition::gazebo::systems::WheelSlip")
