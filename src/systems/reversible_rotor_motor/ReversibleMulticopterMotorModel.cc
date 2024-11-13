#include "ReversibleMulticopterMotorModel.hh"

#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Vector3.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ReversibleMulticopterMotorModelPrivate
{
  /// \brief Callback for actuator commands.
  public: void OnActuatorMsg(const msgs::Actuators &_msg);

  /// \brief Apply link forces and moments based on propeller state.
  public: void UpdateForcesAndMoments(EntityComponentManager &_ecm);

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Name of the robot.
  public: std::string robot_name = "";

  /// \brief Topic for actuator commands.
  public: std::string commandSubTopic = "";

  /// \brief Motor number for reference in the message
  public: int motorNumber = 0;

  /// \brief Maximum rotational velocity command with units of rad/s.
  public: double maxRotVelocity = 838.0;

  /// \brief Thrust coefficient for propeller with units of N / (rad/s)^2.
  public: double motorConstant = 8.54858e-06;

  /// \brief Moment constant for computing drag torque with units of N*m / (rad/s)^2.
  public: double momentConstant = 1.6e-3;

  /// \brief Reference input to motor, normalized between -1 and 1.
  public: double refMotorInput = 0.0;

  /// \brief Rotor drag coefficient for drag torque with units of N*m / (rad/s) * (m/s).
  public: double rotorDragCoefficient = 0.02;

  /// \brief Received Actuators message. This is nullopt if no message has been
  public: std::optional<msgs::Actuators> recvdActuatorsMsg;

  /// \brief Mutex to protect recvdActuatorsMsg.
  public: std::mutex recvdActuatorsMsgMutex;
  
  /// \brief Gazebo communication node.
  public: transport::Node node;
};

//////////////////////////////////////////////////
ReversibleMulticopterMotorModel::ReversibleMulticopterMotorModel()
  : dataPtr(std::make_unique<ReversibleMulticopterMotorModelPrivate>())
{
}

//////////////////////////////////////////////////
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

  // Get params from SDF
  if (sdfClone->HasElement("jointName"))
  {
    std::string jointName = sdfClone->Get<std::string>("jointName");
    this->dataPtr->jointEntity = this->dataPtr->model.JointByName(_ecm, jointName);
  }

  if (sdfClone->HasElement("linkName"))
  {
    std::string linkName = sdfClone->Get<std::string>("linkName");
    this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
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

  sdfClone->Get<double>("maxRotVelocity", this->dataPtr->maxRotVelocity, this->dataPtr->maxRotVelocity);
  sdfClone->Get<double>("motorConstant", this->dataPtr->motorConstant, this->dataPtr->motorConstant);
  sdfClone->Get<double>("momentConstant", this->dataPtr->momentConstant, this->dataPtr->momentConstant);
  sdfClone->Get<double>("rotorDragCoefficient", this->dataPtr->rotorDragCoefficient, this->dataPtr->rotorDragCoefficient);

  // Subscribe to actuator command messages
  std::string topic = transport::TopicUtils::AsValidTopic( 
    this->dataPtr->robot_name + "/" + this->dataPtr->commandSubTopic
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

//////////////////////////////////////////////////
void ReversibleMulticopterMotorModel::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ReversibleMulticopterMotorModel::PreUpdate");

  if (_info.paused)
    return;

  if (this->dataPtr->jointEntity == kNullEntity ||
      this->dataPtr->linkEntity == kNullEntity)
    return;

  this->dataPtr->UpdateForcesAndMoments(_ecm);
}

//////////////////////////////////////////////////
void ReversibleMulticopterMotorModelPrivate::OnActuatorMsg(
    const msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
  this->recvdActuatorsMsg = _msg;
}

//////////////////////////////////////////////////
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

  // Check if message has value
  if (msg.has_value()) {

    // Check if message has velocity field
    if (msg->velocity_size() != 0) {
      
      if (this->motorNumber > msg->velocity_size() - 1) {
        gzerr << "You tried to access index " << this->motorNumber
          << " of the Actuator velocity array which is of size "
          << msg->velocity_size() << std::endl;
        return;
      } else {
        auto speed = msg->velocity(this->motorNumber);
        this->refMotorInput = std::max(
          std::min(speed, this->maxRotVelocity), 
          -this->maxRotVelocity
        );
      }
    // Check if message has normalized field
    } else if (msg->normalized_size() != 0) {
      if (this->motorNumber > msg->normalized_size() - 1) {
        gzerr << "You tried to access index " << this->motorNumber
          << " of the Actuator normalized array which is of size "
          << msg->normalized_size() << std::endl;
        return;
      } else {
        auto speed = msg->normalized(this->motorNumber);
        this->refMotorInput = this->maxRotVelocity * std::max(
          std::min(speed, 1.0), 
          -1.0
        );
      } 
    }
  }

  // Calculate thrust and torque based on motor input
  double motorRotVel = this->refMotorInput * this->maxRotVelocity;
  double thrust = motorRotVel * motorRotVel * this->motorConstant;
  double torque = motorRotVel * motorRotVel * this->momentConstant;


  // Reverse thrust and torque if motor is spinning in reverse
  if (motorRotVel < 0)
  {
    thrust = -thrust;
    torque = -torque;
  }

  // Calculate rotor drag torque based on velocity and rotor drag coefficient
  double dragTorque = -motorRotVel * this->rotorDragCoefficient;

  sim::Link link(this->linkEntity);
  link.AddWorldForce(_ecm, gz::math::Vector3d(0, 0, thrust));
  link.AddWorldForce(_ecm, gz::math::Vector3d(0, 0, torque + dragTorque));
}

GZ_ADD_PLUGIN(ReversibleMulticopterMotorModel,
                    System,
                    ReversibleMulticopterMotorModel::ISystemConfigure,
                    ReversibleMulticopterMotorModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ReversibleMulticopterMotorModel,
                          "gz::sim::systems::ReversibleMulticopterMotorModel")
