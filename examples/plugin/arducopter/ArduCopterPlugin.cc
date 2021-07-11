/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <functional>
#include <fcntl.h>

#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

#include <mutex>
#include <string>
#include <vector>
#include <chrono>

#include <sdf/sdf.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Filter.hh>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"
#include "ignition/math/PID.hh"
#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include "ArduCopterPlugin.hh"

#define MAX_MOTORS 255

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  float motorSpeed[MAX_MOTORS];
};

/// \brief Flight Dynamics Model packet that is sent back to the ArduCopter
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NED frame
  double velocityXYZ[3];

  /// \brief Model position in NED frame
  double positionXYZ[3];
};

/// \brief Rotor class
class Rotor
{
  /// \brief Constructor
  public: Rotor()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;
    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

  /// \brief rotor id
  public: int id = 0;

  /// \brief Max rotor propeller RPM.
  public: double maxRpm = 838.0;

  /// \brief Next command to be applied to the propeller
  public: double cmd = 0;

  /// \brief Velocity PID for motor control
  public: ignition::math::PID pid;

  /// \brief Control propeller joint.
  public: std::string jointName;

  /// \brief Control propeller joint.
  public: Entity joint;

  /// \brief direction multiplier for this rotor
  public: double multiplier = 1;

  /// \brief unused coefficients
  public: double rotorVelocitySlowdownSim;
  public: double frequencyCutoff;
  public: double samplingRate;
  public: ignition::math::OnePole<double> velocityFilter;

  public: static double kDefaultRotorVelocitySlowdownSim;
  public: static double kDefaultFrequencyCutoff;
  public: static double kDefaultSamplingRate;
};

double Rotor::kDefaultRotorVelocitySlowdownSim = 10.0;
double Rotor::kDefaultFrequencyCutoff = 5.0;
double Rotor::kDefaultSamplingRate = 0.2;

class ignition::gazebo::systems::ArduCopterPluginPrivate
{   
   /// \brief Update PID Joint controllers.
   /// \param[in] _dt time step size since last update.
   public: void ApplyMotorForces(const double _dt, EntityComponentManager &_ecm);

   /// \brief Reset PID Joint controllers.
   public: void ResetPIDs();

   /// \brief Receive motor commands from ArduCopter
   public: void ReceiveMotorCommand();

   /// \brief Send state to ArduCopter
   public: void SendState(const UpdateInfo &_info, const EntityComponentManager &_ecm);

   /// \brief Bind to an adress and port
   /// \param[in] _address Address to bind to.
   /// \param[in] _port Port to bind to.
   /// \return True on success.
   public: bool Bind(const char *_address, const uint16_t _port)
   {
     struct sockaddr_in sockaddr;
     this->MakeSockAddr(_address, _port, sockaddr);

     if (bind(this->handle, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
     {
       shutdown(this->handle, 0);
       #ifdef _WIN32
       closesocket(this->handle);
       #else
       close(this->handle);
       #endif
       return false;
     }
     return true;
   }

   /// \brief Make a socket
   /// \param[in] _address Socket address.
   /// \param[in] _port Socket port
   /// \param[out] _sockaddr New socket address structure.
   public: void MakeSockAddr(const char *_address, const uint16_t _port,
     struct sockaddr_in &_sockaddr)
   {
     memset(&_sockaddr, 0, sizeof(_sockaddr));

     #ifdef HAVE_SOCK_SIN_LEN
       _sockaddr.sin_len = sizeof(_sockaddr);
     #endif

     _sockaddr.sin_port = htons(_port);
     _sockaddr.sin_family = AF_INET;
     _sockaddr.sin_addr.s_addr = inet_addr(_address);
   }

   /// \brief Receive data
   /// \param[out] _buf Buffer that receives the data.
   /// \param[in] _size Size of the buffer.
   /// \param[in] _timeoutMS Milliseconds to wait for data.
   public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
   {
     fd_set fds;
     struct timeval tv;

     FD_ZERO(&fds);
     FD_SET(this->handle, &fds);

     tv.tv_sec = _timeoutMs / 1000;
     tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

     if (select(this->handle+1, &fds, NULL, NULL, &tv) != 1)
     {
        return -1;
     }

     #ifdef _WIN32
     return recv(this->handle, reinterpret_cast<char *>(_buf), _size, 0);
     #else
     return recv(this->handle, _buf, _size, 0);
     #endif
   }

   /// \brief interface to the model
   public: Model model;

   public: ignition::gazebo::Entity modelLink{ignition::gazebo::kNullEntity};

   /// \brief array of propellers
   public: std::vector<Rotor> rotors;

   /// \brief keep track of controller update sim-time.
   public: std::chrono::steady_clock::duration lastControllerUpdateTime;

   /// \brief Controller update mutex.
   public: std::mutex mutex;

   /// \brief model name
   public: std::string modelName;

   /// \brief Socket handle
   public: int handle;

   /// \brief false before ArduCopter controller is online
   /// to allow gazebo to continue without waiting
   public: bool arduCopterOnline;

   /// \brief number of times ArduCotper skips update
   public: int connectionTimeoutCount;

   /// \brief number of times ArduCotper skips update
   /// before marking ArduCopter offline
   public: int connectionTimeoutMaxCount;

   /// \brief Latest update info
   public: UpdateInfo updateInfo;

   public: 
      std::string imuName;
      bool imuInitialized;
      ignition::transport::Node node;
      ignition::msgs::IMU imuMsg;
      bool imuMsgValid;
      std::mutex imuMsgMutex;
};

//////////////////////////////////////////////////////////////////////////////
ArduCopterPlugin::ArduCopterPlugin()
  : dataPtr(new ArduCopterPluginPrivate)
{ 
  // socket
  this->dataPtr->handle = socket(AF_INET, SOCK_DGRAM /*SOCK_STREAM*/, 0);
  #ifndef _WIN32
  // Windows does not support FD_CLOEXEC
  fcntl(this->dataPtr->handle, F_SETFD, FD_CLOEXEC);
  #endif
  int one = 1;
  setsockopt(this->dataPtr->handle, IPPROTO_TCP, TCP_NODELAY,
      reinterpret_cast<const char *>(&one), sizeof(one));

  if (!this->dataPtr->Bind("127.0.0.1", 9002))
  {
    ignerr << "failed to bind with 127.0.0.1:9002, aborting plugin.\n";
    return;
  }

  this->dataPtr->arduCopterOnline = false;

  setsockopt(this->dataPtr->handle, SOL_SOCKET, SO_REUSEADDR,
     reinterpret_cast<const char *>(&one), sizeof(one));

  #ifdef _WIN32
  u_long on = 1;
  ioctlsocket(this->dataPtr->handle, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
  #else
  fcntl(this->dataPtr->handle, F_SETFL,
      fcntl(this->dataPtr->handle, F_GETFL, 0) | O_NONBLOCK);
  #endif
}

//////////////////////////////////////////////////////////////////////////////
ArduCopterPlugin::~ArduCopterPlugin()
{
}

//////////////////////////////////////////////////////////////////////////////
void ArduCopterPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{  
   IGN_ASSERT(_sdf, "ArduCopterPlugin _sdf pointer is null");

   this->dataPtr->model = ignition::gazebo::Model(_entity);

   // per rotor
   if(_sdf->HasElement("rotor"))
   {
      sdf::ElementPtr rotorSDF = _sdf->Clone();

      while(rotorSDF)
      {  
         Rotor rotor;

         if(rotorSDF->HasAttribute("id"))
         {  
            rotor.id = rotorSDF->Get<int>("id");
         }
         else
         {
            rotor.id= this->dataPtr->rotors.size();
            ignwarn << "id attribute not specified, use order parsed [" << rotor.id <<"].\n";
         }

         if(rotorSDF->HasElement("jointName"))
         {  
            rotor.jointName = rotorSDF->Get<std::string>("jointName");
         }
         else
         {  
            ignerr << "Please specify a jointName," << "where the rotor is attached.\n";
         }

         // Get the pointer to the joint.
         rotor.joint = this->dataPtr->model.JointByName(_ecm, rotor.jointName);
         if(!rotor.joint)
         {
            ignerr << "Couldn't find specified joint [" << rotor.jointName <<"]. This plugin will not run.\n";
            return; 
         }

         if(rotorSDF->HasElement("turningDirection"))
         {
            std::string turningDirection = rotorSDF->Get<std::string>("turningDirection");
            if(turningDirection == "cw")
            {
               rotor.multiplier = -1;
            }
            else if(turningDirection == "ccw")
            {
               rotor.multiplier = 1;
            }
            else
            {
               igndbg << "not string, check turningDirection as float\n";
               rotor.multiplier = rotorSDF->Get<double>("turningDirection");
            }
          }

          else
          {  
             rotor.multiplier = 1;
             ignerr << "Please specify a turning" << " direction multiplier ('cw' or 'ccw'). Default 'ccw'.\n";
          }

          _sdf->Get<double>("rotorVelocitySlowdownSim", rotor.rotorVelocitySlowdownSim, rotor.rotorVelocitySlowdownSim);

          if (ignition::math::equal(rotor.rotorVelocitySlowdownSim, 0.0))
          {  
             ignerr << "rotor for joint [" << rotor.jointName << "] rotorVelocitySlowdownSim is zero," << " aborting plugin.\n";
             return;
          }
          _sdf->Get<double>("frequencyCutoff",rotor.frequencyCutoff, rotor.frequencyCutoff);
          _sdf->Get<double>("samplingRate",rotor.samplingRate, rotor.samplingRate);

          // use ignition::math::Filter
          rotor.velocityFilter.Fc(rotor.frequencyCutoff, rotor.samplingRate);

          // initialize filter to zero value
          rotor.velocityFilter.Set(0.0);

          // note to use this
          // rotorVelocityFiltered = velocityFilter.Process(rotorVelocityRaw);

          // Overload the PID parameters if they are available.
          double param;

          _sdf->Get<double>("vel_p_gain", param, rotor.pid.PGain());
          rotor.pid.SetPGain(param);
          _sdf->Get<double>("vel_i_gain", param, rotor.pid.IGain());
          rotor.pid.SetIGain(param);
          _sdf->Get<double>("vel_d_gain", param, rotor.pid.DGain());
          rotor.pid.SetDGain(param);
          _sdf->Get<double>("vel_i_max", param, rotor.pid.IMax());
          rotor.pid.SetIMax(param);
          _sdf->Get<double>("vel_i_min", param, rotor.pid.IMin());
          rotor.pid.SetIMin(param);
          _sdf->Get<double>("vel_cmd_max", param, rotor.pid.CmdMax());
          rotor.pid.SetCmdMax(param);
          _sdf->Get<double>("vel_cmd_min", param, rotor.pid.CmdMin());
          rotor.pid.SetCmdMin(param);

          // set pid initial command
          rotor.pid.SetCmd(0.0);

          this->dataPtr->rotors.push_back(rotor);
          rotorSDF = rotorSDF->GetNextElement("rotor");

          this->dataPtr->imuName = _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
      }
   }
   // Missed update count before we declare arduCopterOnline status false
   _sdf->Get<int>("connectionTimeoutMaxCount",this->dataPtr->connectionTimeoutMaxCount, this->dataPtr->connectionTimeoutMaxCount);

   ignlog <<"ArduCopter ready to fly. The force will be with you"<< std::endl;
}

//////////////////////////////////////////////////////////////////////////////
void ArduCopterPlugin::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
   IGN_PROFILE("ArduCopterPlugin::PreUpdtae");
   IGN_PROFILE("Update");
   this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);
   std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

   if(_info.simTime > this->dataPtr->lastControllerUpdateTime)
   {
      IGN_PROFILE_BEGIN("ReceiveMotorCommand");
      this->dataPtr->ReceiveMotorCommand();
      IGN_PROFILE_END();

      if(this->dataPtr->arduCopterOnline)
      {
         IGN_PROFILE_BEGIN("ApplyMotorForces");
         this->dataPtr->ApplyMotorForces(std::chrono::duration_cast<std::chrono::duration<double> >(_info.simTime -
         this->dataPtr->lastControllerUpdateTime).count(), _ecm);
         IGN_PROFILE_END();
         IGN_PROFILE_BEGIN("SendState");
         this->dataPtr->SendState(_info, _ecm);
         IGN_PROFILE_END();
      }
   }
   this->dataPtr->lastControllerUpdateTime = _info.simTime;

   if(!this->dataPtr->imuInitialized)
   {
      // Set unconditionally because we're only going to try this once.
      this->dataPtr->imuInitialized = true;
      std::string imuTopicName;

      auto imuEntity = _ecm.EntityByComponents(
      components::Name(this->dataPtr->imuName),
      components::Imu(),
      components::ParentEntity(this->dataPtr->modelLink));
      imuTopicName = ignition::gazebo::scopedName(imuEntity, _ecm), "/imu";

      if(!imuTopicName.empty())
      {
        ignerr << "[" << this->dataPtr->modelName << "] "
               << "imu_sensor [" << this->dataPtr->imuName
               << "] not found, abort ArduCopter plugin." << "\n";
        return;
      }

      auto imuCb = std::function<void(const msgs::IMU &_msg)>(
      [this](const auto &_msg)
      {
         std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
         this->dataPtr->imuMsg = _msg;
         this->dataPtr->imuMsgValid = true;
      });

      this->dataPtr->node.Subscribe(imuTopicName, imuCb);
}
   IGN_PROFILE_END();
}

//////////////////////////////////////////////////////////////////////////////
void ArduCopterPluginPrivate::ResetPIDs()
{  
   // Reset velocity PID for rotors
   for (size_t i = 0; i < this->rotors.size(); ++i)
   {  
      this->rotors[i].cmd = 0;
      // this->dataPtr->rotors[i].pid.Reset();
   }
}

//////////////////////////////////////////////////////////////////////////////
void ArduCopterPluginPrivate::ApplyMotorForces(const double _dt, EntityComponentManager &_ecm)
{
   // update velocity PID for rotors and apply force to joint
   ignition::gazebo::UpdateInfo &_info(this->updateInfo);
   for (size_t i = 0; i < this->rotors.size(); ++i)
   {
      const double velTarget = this->rotors[i].multiplier * this->rotors[i].cmd / this->rotors[i].rotorVelocitySlowdownSim;
      const auto vel = _ecm.Component<components::JointVelocity>(this->rotors[i].joint)->Data().at(0);
      const double error = vel - velTarget;
      const double force = this->rotors[i].pid.Update(error, _info.dt);
      auto forceComp =
        _ecm.Component<components::JointForceCmd>(this->rotors[i].joint);
      if (forceComp == nullptr)
      {
         _ecm.CreateComponent(this->rotors[i].joint,
                           components::JointForceCmd({force}));
      } 
      else
      {
         forceComp->Data()[0] = force;
      }
      //_ecm.SetComponentData(this->rotors[i].joint, components::JointForceCmd({force}));
    }
}

//////////////////////////////////////////////////////////////////////////////
void ArduCopterPluginPrivate::ReceiveMotorCommand()
{
   // Added detection for whether ArduCopter is online or not.
   // If ArduCopter is detected (receive of fdm packet from someone),
   // then socket receive wait time is increased from 1ms to 1 sec
   // to accomodate network jitter.
   // If ArduCopter is not detected, receive call blocks for 1ms
   // on each call.
   // Once ArduCopter presence is detected, it takes this many
   // missed receives before declaring the FCS offline.

   ServoPacket pkt;
   int waitMs = 1;
   if (this->arduCopterOnline)
   {
     // increase timeout for receive once we detect a packet from
     // ArduCopter FCS.
     waitMs = 1000;
   }
   else
   {
     // Otherwise skip quickly and do not set control force.
     waitMs = 1;
   }
   ssize_t recvSize = this->Recv(&pkt, sizeof(ServoPacket), waitMs);
   ssize_t expectedPktSize = sizeof(pkt.motorSpeed[0])*this->rotors.size();

   if ((recvSize == -1) || (recvSize < expectedPktSize))
   {
      if (recvSize != -1)
      {  
         ignerr << "received bit size (" << recvSize << ") to small,"
                << " controller expected size (" << expectedPktSize << ").\n";
      }

       if (this->arduCopterOnline)
       {
          ignwarn << "Broken ArduCopter connection, count ["
                 << this->connectionTimeoutCount
                 << "/" << this->connectionTimeoutMaxCount
                 << "]\n";
          if (++this->connectionTimeoutCount > this->connectionTimeoutMaxCount)
          {
             this->connectionTimeoutCount = 0;
             this->arduCopterOnline = false;
             ignwarn << "Broken ArduCopter connection, resetting motor control.\n";
             this->ResetPIDs();
          }
        }
      }
      else
      {
        if (!this->arduCopterOnline)
        {
           igndbg << "ArduCopter controller online detected.\n";
           // made connection, set some flags
           this->connectionTimeoutCount = 0;
           this->arduCopterOnline = true;
        }

        // compute command based on requested motorSpeed
        for (unsigned i = 0; i < this->rotors.size(); ++i)
        {
           if (i < MAX_MOTORS)
           {
              // std::cout << i << ": " << pkt.motorSpeed[i] << "\n";
              this->rotors[i].cmd = this->rotors[i].maxRpm *
              pkt.motorSpeed[i];
            }
            else
            {
               ignerr << "too many motors, skipping [" << i
               << " > " << MAX_MOTORS << "].\n";
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
void ArduCopterPluginPrivate::SendState(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
   // send_fdm
   fdmPacket pkt;
   pkt.timestamp = std::chrono::duration<double>(_info.simTime).count();

   // asssumed that the imu orientation is:
   //   x forward
   //   y right
   //   z down

   // get linear acceleration in body frame
   // get angular velocity in body frame
   
   // NEW
   ignition::msgs::IMU imuMsg;
   {
     std::lock_guard<std::mutex> lock(this->imuMsgMutex);

     if(!this->imuMsgValid)
     {
       return;
     }
     imuMsg = this->imuMsg;
   }

   pkt.imuLinearAccelerationXYZ[0] = imuMsg.linear_acceleration().x();
   pkt.imuLinearAccelerationXYZ[1] = imuMsg.linear_acceleration().y();
   pkt.imuLinearAccelerationXYZ[2] = imuMsg.linear_acceleration().z();

   pkt.imuAngularVelocityRPY[0] = imuMsg.angular_velocity().x();
   pkt.imuAngularVelocityRPY[1] = imuMsg.angular_velocity().y();
   pkt.imuAngularVelocityRPY[2] = imuMsg.angular_velocity().z();

   // get inertial pose and velocity
   // position of the quadrotor in world frame
   // this position is used to calcualte bearing and distance
   // from starting location, then use that to update gps position.
   // The algorithm looks something like below (from ArduCopter helper
   // libraries):
   //   bearing = to_degrees(atan2(position.y, position.x));
   //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
   //   (self.latitude, self.longitude) = util.gps_newpos(
   //    self.home_latitude, self.home_longitude, bearing, distance)
   // where xyz is in the NED directions.
   // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
   // around.
   // orientation of the quadrotor in world NED frame -
   // assuming the world NED frame has xyz mapped to NED,
   // imuLink is NED - z down

   // gazeboToNED brings us from gazebo model: x-forward, y-right, z-down
   // to the aerospace convention: x-forward, y-left, z-up
   ignition::math::Pose3d gazeboToNED(0, 0, 0, IGN_PI, 0, 0);

   // model world pose brings us to model, x-forward, y-left, z-up
   // adding gazeboToNED gets us to the x-forward, y-right, z-down
   const ignition::gazebo::components::WorldPose* pComp = _ecm.Component<ignition::gazebo::components::WorldPose>(this->modelLink);
   ignition::math::Pose3d worldToModel = gazeboToNED +  pComp->Data();

   // get transform from world NED to Model frame
   ignition::math::Pose3d NEDToModel = worldToModel - gazeboToNED;

   // N
   pkt.positionXYZ[0] = NEDToModel.Pos().X();

   // E
   pkt.positionXYZ[1] = NEDToModel.Pos().Y();
 
   // D
   pkt.positionXYZ[2] = NEDToModel.Pos().Z();

   // imuOrientationQuat is the rotation from world NED frame
   // to the quadrotor frame.
   pkt.imuOrientationQuat[0] = NEDToModel.Rot().W();
   pkt.imuOrientationQuat[1] = NEDToModel.Rot().X();
   pkt.imuOrientationQuat[2] = NEDToModel.Rot().Y();
   pkt.imuOrientationQuat[3] = NEDToModel.Rot().Z();

   // Get NED velocity in body frame *
   // or...
   // Get model velocity in NED frame
   const ignition::gazebo::components::WorldLinearVelocity* vComp = _ecm.Component<ignition::gazebo::components::WorldLinearVelocity>(this->modelLink);
   ignition::math::Vector3d velGazeboWorldFrame = vComp->Data();

   ignition::math::Vector3d velNEDFrame = gazeboToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
   pkt.velocityXYZ[0] = velNEDFrame.X();
   pkt.velocityXYZ[1] = velNEDFrame.Y();
   pkt.velocityXYZ[2] = velNEDFrame.Z();

   struct sockaddr_in sockaddr;
   this->MakeSockAddr("127.0.0.1", 9003, sockaddr);

   ::sendto(this->handle,
           reinterpret_cast<raw_type *>(&pkt),
           sizeof(pkt), 0,
           (struct sockaddr *)&sockaddr, sizeof(sockaddr));
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::ArduCopterPlugin,
                    ignition::gazebo::System,
                    ignition::gazebo::systems::ArduCopterPlugin::ISystemConfigure,
                    ignition::gazebo::systems::ArduCopterPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(ignition::gazebo::systems::ArduCopterPlugin,"ArduCopterPlugin")