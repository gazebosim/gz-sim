#include "ReversibleMulticopterMotorModel.hh"

#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ReversibleMulticopterMotorModelPrivate
{
  public: void OnActuatorMsg(const msgs::Actuators &_msg);
  public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);
  public: Entity jointEntity;
  public: Entity linkEntity;
  public: Entity parentLinkEntity; 
  public: std::string jointName = ""; 
  public: std::string linkName = "";
  public: std::string parentLinkName = "";
  public: Model model{kNullEntity};
  public: std::string robot_name = "";
  public: std::string commandSubTopic = "";
  public: int motorNumber = 0;
  public: double maxRotVelocity = 838.0;
  public: double motorInputVel = 0.0;
  public: double simVelSlowDown = 10.0;

  public: std::vector<double> TorquePolynomial = {0.0, 0.0, 0.0, 0.0};
  public: std::vector<double> ThrustPolynomial = {0.0, 0.0, 0.0, 0.0};

  public: std::optional<msgs::Actuators> recvdActuatorsMsg;
  public: std::mutex recvdActuatorsMsgMutex;
  public: transport::Node node;
};

ReversibleMulticopterMotorModel::ReversibleMulticopterMotorModel()
  : dataPtr(std::make_unique<ReversibleMulticopterMotorModelPrivate>())
{
}

void ReversibleMulticopterMotorModel::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ReversibleMulticopterMotorModel plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  if(sdfClone->HasElement("robotName")){
    std::string robotName = sdfClone->Get<std::string>("robotName");
    this->dataPtr->robot_name = robotName;
  }else{
    this->dataPtr->robot_name = "default_robot";
    gzerr << "robotName not specified, using default_robot.\n";
  }

  if (sdfClone->HasElement("jointName"))
  {
    dataPtr->jointName = sdfClone->Get<std::string>("jointName");
    dataPtr->jointEntity = this->dataPtr->model.JointByName(_ecm, dataPtr->jointName);
  }

  if (sdfClone->HasElement("linkName"))
  {
    dataPtr->linkName = sdfClone->Get<std::string>("linkName");
    dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, dataPtr->linkName);
  }

  if (sdfClone->HasElement("commandSubTopic"))
  {
    this->dataPtr->commandSubTopic = sdfClone->Get<std::string>("commandSubTopic");
  }

  if (sdfClone->HasElement("motorNumber"))
  {
    this->dataPtr->motorNumber = sdfClone->Get<unsigned int>("motorNumber");
  } else {
    this->dataPtr->motorNumber = 0;
    gzerr << "motorNumber not specified, using 0.\n";
  }

  if (this->dataPtr->jointName != "") {
    dataPtr->parentLinkName = _ecm.Component<components::ParentLinkName>(dataPtr->jointEntity)->Data();
    dataPtr->parentLinkEntity  = dataPtr->model.LinkByName(_ecm, dataPtr->parentLinkName);
  }

  auto a0Thrust = sdfClone->Get<double>("a0ThrustConstant");
  auto a1Thrust = sdfClone->Get<double>("a1ThrustConstant");
  auto a2Thrust = sdfClone->Get<double>("a2ThrustConstant");
  auto a3Thrust = sdfClone->Get<double>("a3ThrustConstant");

  this->dataPtr->ThrustPolynomial = {a0Thrust, a1Thrust, a2Thrust, a3Thrust};

  auto a0Torque = sdfClone->Get<double>("a0TorqueConstant");
  auto a1Torque = sdfClone->Get<double>("a1TorqueConstant");
  auto a2Torque = sdfClone->Get<double>("a2TorqueConstant");
  auto a3Torque = sdfClone->Get<double>("a3TorqueConstant");

  this->dataPtr->TorquePolynomial = {a0Torque, a1Torque, a2Torque, a3Torque};


  dataPtr->simVelSlowDown = sdfClone->Get<double>("simVelSlowDown");

  std::string topic = transport::TopicUtils::AsValidTopic( 
    "/" +  this->dataPtr->robot_name + "/" + this->dataPtr->commandSubTopic
  );
  if (topic.empty())
  {
    gzerr << "Failed to create topic for command subscription." << std::endl;
    return;
  }
  else
  {
    gzdbg << "Listening to topic: " << topic << std::endl;
  }
  this->dataPtr->node.Subscribe(topic,
      &ReversibleMulticopterMotorModelPrivate::OnActuatorMsg, this->dataPtr.get());
}

void ReversibleMulticopterMotorModel::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ReversibleMulticopterMotorModel::PreUpdate");

  if (_info.paused) {
    return;
  }

  if (this->dataPtr->jointEntity == kNullEntity ||
      this->dataPtr->linkEntity == kNullEntity) {
    gzerr << "Joint or link entity is null." << std::endl;
    return;
  }

  if (!_ecm.Component<components::JointVelocityCmd>(this->dataPtr->jointEntity))
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity, components::JointVelocityCmd({0}));
  }

  this->dataPtr->UpdateForcesAndMoments(_ecm);
}

void ReversibleMulticopterMotorModelPrivate::OnActuatorMsg(
    const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
  this->recvdActuatorsMsg = _msg;
}

void ReversibleMulticopterMotorModelPrivate::UpdateForcesAndMoments(
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ReversibleMulticopterMotorModelPrivate::UpdateForcesAndMoments");
  
  auto actuatorMsgComp = _ecm.Component<components::Actuators>(this->model.Entity());
  std::optional<msgs::Actuators> msg;

  if (actuatorMsgComp)
  {
    msg = actuatorMsgComp->Data();
  } else {
    std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);

    if (this->recvdActuatorsMsg.has_value())
    {
      msg = *this->recvdActuatorsMsg;
      this->recvdActuatorsMsg.reset();
    }
  }


  if (msg.has_value()) {
    if (msg->velocity_size() > this->motorNumber) {
      this->motorInputVel = std::clamp(
        msg->velocity(this->motorNumber),
        -this->maxRotVelocity, 
        this->maxRotVelocity
      );
    } else if (msg->normalized_size() > this->motorNumber) {
      this->motorInputVel = std::clamp(msg->normalized(this->motorNumber), -1.0, 1.0) * this->maxRotVelocity;  
    }
  }
  
  sim::Link link(this->linkEntity);
  const auto worldPose = link.WorldPose(_ecm);
  using Vector3 = math::Vector3d;

  // Compute thrust according to the polynomial we have defined
  double thrust = 0.0;
  for (unsigned int i = 0; i < this->ThrustPolynomial.size(); i++) {
    thrust += this->ThrustPolynomial[i] * std::pow(this->motorInputVel, i);
  }

  link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));

  double torque = 0.0;

  for (unsigned int i = 0; i < this->TorquePolynomial.size(); i++) {
    torque += this->TorquePolynomial[i] * std::pow(this->motorInputVel, i);
  }

  link.AddWorldForce(_ecm, worldPose->Rot().RotateVector(Vector3(0, 0, torque)));

  const auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(
      this->jointEntity); 
  *jointVelCmd = components::JointVelocityCmd({this->motorInputVel / this->simVelSlowDown});
}

GZ_ADD_PLUGIN(ReversibleMulticopterMotorModel,
                    System,
                    ReversibleMulticopterMotorModel::ISystemConfigure,
                    ReversibleMulticopterMotorModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ReversibleMulticopterMotorModel,
                          "gz::sim::systems::ReversibleMulticopterMotorModel")
