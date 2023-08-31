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

#include "ModelCommandAPI.hh"

#include <gz/msgs/serialized.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <string>
#include <vector>
#include <map>

#include <sdf/AirPressure.hh>
#include <sdf/Altimeter.hh>
#include <sdf/Camera.hh>
#include <sdf/Imu.hh>
#include <sdf/Lidar.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Noise.hh>
#include <sdf/Sensor.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/AirPressureSensor.hh>
#include <gz/sim/components/Altimeter.hh>
#include <gz/sim/components/Camera.hh>
#include <gz/sim/components/ChildLinkName.hh>
#include <gz/sim/components/GpuLidar.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Magnetometer.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RgbdCamera.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
/// \brief Get the name of the world being used by calling
/// `/gazebo/worlds` service.
/// \return The name of the world if service is available,
/// an empty string otherwise.
std::string getWorldName()
{
  // Create a transport node.
  transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/gazebo/worlds"};

  // Request and block
  msgs::StringMsg_V res;

  if (!node.Request(service, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return "";
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return "";
  }

  return res.data().Get(0);
}

//////////////////////////////////////////////////
/// \brief Get entity info: name and entity ID
/// \param[in] _entity Entity to get info
/// \param[in] _ecm Entity component manager
/// \return "<entity name> [<entity ID>]"
std::string entityInfo(Entity _entity, const EntityComponentManager &_ecm)
{
  std::string info;

  const auto nameComp = _ecm.Component<components::Name>( _entity);
  if (nameComp)
  {
    info += nameComp->Data() + " ";
  }
  info += "[" + std::to_string(_entity) + "]";
  return info;
}

//////////////////////////////////////////////////
/// \brief Get entity info: name and entity ID
/// \param[in] _entity Name of entity to get info
/// \param[in] _ecm Entity component manager
/// \return "<entity name> [<entity ID>]"
std::string entityInfo(const std::string &_name,
    const EntityComponentManager &_ecm)
{
  std::string info{_name};

  auto entity = _ecm.EntityByComponents(components::Name(_name));
  if (kNullEntity != entity)
  {
    info += " [" + std::to_string(entity) + "]";
  }
  return info;
}

//////////////////////////////////////////////////
/// \brief Get pose info in a standard way
/// \param[in] _pose Pose to print
/// \param[in] _spaces Number of spaces to indent for every line
/// \return Pose formatted in a standard way
std::string poseInfo(math::Pose3d _pose, int _spaces)
{
  return
    std::string(_spaces, ' ') + "[" + std::to_string(_pose.X()) + " "
                  + std::to_string(_pose.Y()) + " "
                  + std::to_string(_pose.Z()) + "]\n" +
    std::string(_spaces, ' ') + "[" + std::to_string(_pose.Roll()) + " "
                  + std::to_string(_pose.Pitch()) + " "
                  + std::to_string(_pose.Yaw()) + "]";
}

//////////////////////////////////////////////////
/// \brief Print pose info about an entity.
/// \param[in] _entity Entity to print pose information for. Nothing is
/// printed if the entity lack a pose component.
/// \param[in] _ecm The entity component manager.
/// \param[in] _spaces Number of spaces to indent for every line
void printPose(const uint64_t _entity, const EntityComponentManager &_ecm,
    int _spaces)
{
  const auto poseComp = _ecm.Component<components::Pose>(_entity);
  if (poseComp)
  {
    std::cout << std::string(_spaces, ' ')
      << "- Pose [ XYZ (m) ] [ RPY (rad) ]:" << std::endl
      << poseInfo(poseComp->Data(), _spaces + 2) << std::endl;
  }
}

//////////////////////////////////////////////////
/// \brief Print noise information.
/// \param[in] _noise Noise to print.
/// \param[in] _spaces Number of spaces to indent for every line.
void printNoise(const sdf::Noise &_noise, int _spaces,
    const std::string &_units)
{
  std::string units = "";
  if (!_units.empty())
    units = std::string(" (") + _units + ")";

  std::cout << std::string(_spaces, ' ') << "- Mean" << units << ": "
    << _noise.Mean() << "\n"
    << std::string(_spaces, ' ') << "- Bias mean" << units << ": "
    << _noise.BiasMean() << "\n"
    << std::string(_spaces, ' ') << "- Standard deviation" << units << ": "
    << _noise.StdDev() << "\n"
    << std::string(_spaces, ' ') << "- Bias standard deviation" << units << ": "
    << _noise.BiasStdDev() << "\n"
    << std::string(_spaces, ' ') << "- Precision: "
    << _noise.Precision() << "\n"
    << std::string(_spaces, ' ') << "- Dynamic bias standard deviation"
    << units << ": "
    << _noise.DynamicBiasStdDev() << "\n"
    << std::string(_spaces, ' ') << "- Dynamic bias correlation time (s): "
    << _noise.DynamicBiasCorrelationTime() << std::endl;
}

//////////////////////////////////////////////////
/// \brief Print info about an air pressure sensor.
/// \param[in] _entity Entity to print information for. Nothing is
/// printed if the entity is not an air pressure sensor.
/// \param[in] _ecm The entity component manager.
/// \param[in] _spaces Number of spaces to indent for every line
void printAirPressure(const uint64_t _entity,
    const EntityComponentManager &_ecm, int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::AirPressureSensor>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::AirPressure *air = sensor.AirPressureSensor();

  std::cout << std::string(_spaces, ' ') << "- Reference altitude (m): "
    << air->ReferenceAltitude() << "\n";

  std::cout << std::string(_spaces, ' ') << "- Pressure noise:\n";
  printNoise(air->PressureNoise(), _spaces + 2, "Pa");
}

//////////////////////////////////////////////////
/// \brief Print info about an altimeter sensor.
/// \param[in] _entity Entity to print information for. Nothing is
/// printed if the entity is not an altimeter.
/// \param[in] _ecm The entity component manager.
/// \param[in] _spaces Number of spaces to indent for every line
void printAltimeter(const uint64_t _entity, const EntityComponentManager &_ecm,
    int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::Altimeter>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::Altimeter *altimeter = sensor.AltimeterSensor();

  std::cout << std::string(_spaces, ' ') << "- Vertical position noise:\n";
  printNoise(altimeter->VerticalPositionNoise(), _spaces + 2, "m");

  std::cout << std::string(_spaces, ' ') << "- Vertical velocity noise:\n";
  printNoise(altimeter->VerticalVelocityNoise(), _spaces + 2, "m/s");
}

//////////////////////////////////////////////////
/// \brief Print info about an SDF camera.
/// \param[in] _camera The camera to output.
/// \param[in] _spaces Number of spaces to indent for every line.
void printCamera(const sdf::Camera *_camera, int _spaces)
{
  std::cout << std::string(_spaces, ' ')
    << "- Horizontal field of view (rad): " << _camera->HorizontalFov()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Image width (px): " << _camera->ImageWidth()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Image height (px): " << _camera->ImageHeight()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Near clip (m): " << _camera->NearClip()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Far clip (m): " << _camera->FarClip()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Pixel format: " << _camera->PixelFormatStr()
    << std::endl;

  if (_camera->HasDepthCamera())
  {
    std::cout << std::string(_spaces, ' ')
      << "- Depth near clip (m): " << _camera->DepthNearClip()
      << std::endl;
    std::cout << std::string(_spaces, ' ')
      << "- Depth far clip (m): " << _camera->DepthFarClip()
      << std::endl;
  }

  if (_camera->HasSegmentationType())
  {
    std::cout << std::string(_spaces, ' ')
      << "- Segmentation type: " << _camera->SegmentationType()
      << std::endl;
  }

  if (_camera->HasBoundingBoxType())
  {
    std::cout << std::string(_spaces, ' ')
      << "- Bounding box type: " << _camera->BoundingBoxType()
      << std::endl;
  }

  std::cout << std::string(_spaces, ' ')
    << "- Save frames: " << _camera->SaveFrames()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Save frames path: " << _camera->SaveFramesPath()
    << std::endl;

  std::cout << std::string(_spaces, ' ')
    << "- Image noise:\n";
  printNoise(_camera->ImageNoise(), _spaces + 2, "");

  std::cout << std::string(_spaces, ' ')
    << "- Distortion K1: " << _camera->DistortionK1()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Distortion K2: " << _camera->DistortionK2()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Distortion K3: " << _camera->DistortionK3()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Distortion P1: " << _camera->DistortionP1()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Distortion P2: " << _camera->DistortionP2()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Distortion center: " << _camera->DistortionCenter()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens type: " << _camera->LensType()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens scale to horizontal field of view (rad): "
    << _camera->LensScaleToHfov()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens C1: " << _camera->LensC1()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens C2: " << _camera->LensC2()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens C3: " << _camera->LensC3()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens focal length (m): " << _camera->LensFocalLength()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens function: " << _camera->LensFunction()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens cutoff angle (rad): " << _camera->LensCutoffAngle()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens texture size: " << _camera->LensEnvironmentTextureSize()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens intrinsics Fx: " << _camera->LensIntrinsicsFx()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens intrinsics Fy: " << _camera->LensIntrinsicsFy()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens intrinsics Cx: " << _camera->LensIntrinsicsCx()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens intrinsics Cy: " << _camera->LensIntrinsicsCy()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Lens intrinsics skew: " << _camera->LensIntrinsicsSkew()
    << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Visibility mask: " << _camera->VisibilityMask()
    << std::endl;
}

//////////////////////////////////////////////////
/// \brief Print info about an RGBD camera sensor.
/// \param[in] _entity Entity to print information for. Nothing is
/// printed if the entity is not an RGBD camera.
/// \param[in] _ecm The entity component manager.
/// \param[in] _spaces Number of spaces to indent for every line
void printRgbdCamera(const uint64_t _entity, const EntityComponentManager &_ecm,
    int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::RgbdCamera>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::Camera *camera = sensor.CameraSensor();

  printCamera(camera, _spaces);
}

//////////////////////////////////////////////////
/// \brief Print info about a camera sensor.
/// \param[in] _entity Entity to print information for. Nothing is
/// printed if the entity is not a camera.
/// \param[in] _ecm The entity component manager.
/// \param[in] _spaces Number of spaces to indent for every line
void printCamera(const uint64_t _entity, const EntityComponentManager &_ecm,
    int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::Camera>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::Camera *camera = sensor.CameraSensor();

  printCamera(camera, _spaces);
}

//////////////////////////////////////////////////
/// \brief Print info about an imu sensor.
/// \param[in] _entity Entity to print information for. Nothing is
/// printed if the entity is not an IMU.
/// \param[in] _ecm The entity component manager.
/// \param[in] _spaces Number of spaces to indent for every line
void printImu(const uint64_t _entity, const EntityComponentManager &_ecm,
    int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::Imu>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::Imu *imu = sensor.ImuSensor();

  std::cout << std::string(_spaces, ' ')
    << "- Linear acceleration X-axis noise:\n";
  printNoise(imu->LinearAccelerationXNoise(), _spaces + 2, "m/s^2");
  std::cout << std::string(_spaces, ' ')
    << "- Linear acceleration Y-axis noise:\n";
  printNoise(imu->LinearAccelerationYNoise(), _spaces + 2, "m/s^2");
  std::cout << std::string(_spaces, ' ')
    << "- Linear acceleration Z-axis noise:\n";
  printNoise(imu->LinearAccelerationZNoise(), _spaces + 2, "m/s^2");

  std::cout << std::string(_spaces, ' ')
    << "- Angular velocity X-axis noise:\n";
  printNoise(imu->AngularVelocityXNoise(), _spaces + 2, "rad/s");
  std::cout << std::string(_spaces, ' ')
    << "- Angular velocity Y-axis noise:\n";
  printNoise(imu->AngularVelocityYNoise(), _spaces + 2, "rad/s");
  std::cout << std::string(_spaces, ' ')
    << "- Angular velocity Z-axis noise:\n";
  printNoise(imu->AngularVelocityZNoise(), _spaces + 2, "rad/s");

  std::cout << std::string(_spaces, ' ')
    << "- Gravity direction X [XYZ]: "
    << imu->GravityDirX() << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Gravity direction X parent frame:" << imu->GravityDirXParentFrame()
    << std::endl;

  std::cout << std::string(_spaces, ' ')
    << "- Localization:" << imu->Localization() << std::endl;

  std::cout << std::string(_spaces, ' ')
    << "- Custom RPY: " << imu->CustomRpy() << std::endl;
  std::cout << std::string(_spaces, ' ')
    << "- Custom RPY parent frame:" << imu->CustomRpyParentFrame() << std::endl;

  std::cout << std::string(_spaces, ' ')
    << "- Orientation enabled:" << imu->OrientationEnabled() << std::endl;
}

//////////////////////////////////////////////////
void printGpuLidar(const uint64_t _entity,
    const EntityComponentManager &_ecm, int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::GpuLidar>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::Lidar *lidar = sensor.LidarSensor();

  std::cout << std::string(_spaces, ' ') << "- Range:\n";
  std::cout << std::string(_spaces+2, ' ') << "- Min (m): "
    << lidar->RangeMin() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Max (m): "
    << lidar->RangeMax() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Resolution: "
    << lidar->RangeResolution() << std::endl;

  std::cout << std::string(_spaces, ' ') << "- Horizontal scan:\n";
  std::cout << std::string(_spaces+2, ' ') << "- Samples: "
    << lidar->HorizontalScanSamples() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Resolution: "
    << lidar->HorizontalScanResolution() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Min angle (rad): "
    << lidar->HorizontalScanMinAngle() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Max angle (rad): "
    << lidar->HorizontalScanMaxAngle() << std::endl;

  std::cout << std::string(_spaces, ' ') << "- Vertical scan:\n";
  std::cout << std::string(_spaces+2, ' ') << "- Samples: "
    << lidar->VerticalScanSamples() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Resolution: "
    << lidar->VerticalScanResolution() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Min angle (rad): "
    << lidar->VerticalScanMinAngle() << std::endl;
  std::cout << std::string(_spaces+2, ' ') << "- Max angle (rad): "
    << lidar->VerticalScanMaxAngle() << std::endl;

  std::cout << std::string(_spaces, ' ') << "- Noise:\n";
  printNoise(lidar->LidarNoise(), _spaces + 2, "m");
}

//////////////////////////////////////////////////
void printMagnetometer(const uint64_t _entity,
    const EntityComponentManager &_ecm, int _spaces)
{
  // Get the type and return if the _entity does not have the correct
  // component.
  auto comp = _ecm.Component<components::Magnetometer>(_entity);
  if (!comp)
    return;

  const sdf::Sensor &sensor = comp->Data();
  const sdf::Magnetometer *mag = sensor.MagnetometerSensor();

  std::cout << std::string(_spaces, ' ') << "- X-axis noise:\n";
  printNoise(mag->XNoise(), _spaces + 2, "T");
  std::cout << std::string(_spaces, ' ') << "- Y-axis noise:\n";
  printNoise(mag->YNoise(), _spaces + 2, "T");
  std::cout << std::string(_spaces, ' ') << "- Z-axis noise:\n";
  printNoise(mag->ZNoise(), _spaces + 2, "T");
}

//////////////////////////////////////////////////
// \brief Set the state of a ECM instance with a world snapshot.
// \param _ecm ECM instance to be populated.
// \return boolean indicating if it was able to populate the ECM.
bool populateECM(EntityComponentManager &_ecm)
{
  const std::string world = getWorldName();
  if (world.empty())
  {
    std::cerr << "Command failed when trying to get the world name of "
              << "the running simulation." << std::endl;
    return false;
  }
  // Create a transport node.
  transport::Node node;
  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world/" + world + "/state"};

  std::cout << std::endl << "Requesting state for world [" << world
            << "]..." << std::endl << std::endl;

  // Request and block
  msgs::SerializedStepMap res;

  if (!node.Request(service, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return false;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed"
              << std::endl;
    return false;
  }

  // Instantiate an ECM and populate with data from message
  _ecm.SetState(res.state());
  return true;
}


//////////////////////////////////////////////////
// \brief Print the model information.
// \param[in] _entity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
void printModelInfo(const uint64_t _entity,
               const EntityComponentManager &_ecm)
{
  const auto poseComp =
      _ecm.Component<components::Pose>(_entity);
  const auto nameComp =
      _ecm.Component<components::Name>(_entity);
  if (poseComp && nameComp)
  {
    std::cout << "Model: [" << _entity << "]" << std::endl
              << "  - Name: " << nameComp->Data() << std::endl;
    printPose(_entity, _ecm, 2);
  }
}

//////////////////////////////////////////////////
// \brief Print the model links information.
// \param[in] _modelEntity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
// \param[in] _linkName Link to be printed, if empty, print all links.
void printLinks(const uint64_t _modelEntity,
                const EntityComponentManager &_ecm,
                const std::string &_linkName,
                const std::string &_sensorName,
                int _spaces)
{
  const auto links = _ecm.EntitiesByComponents(
      components::ParentEntity(_modelEntity), components::Link());
  for (const auto &entity : links)
  {
    const auto nameComp = _ecm.Component<components::Name>(entity);

    int spaces = _spaces;

    if (_linkName.length() && _linkName != nameComp->Data())
        continue;

    if (_sensorName.empty())
    {
      std::cout << std::string(spaces, ' ')
        << "- Link [" << entity << "]" << std::endl
        << std::string(spaces + 2, ' ')
        << "- Name: " << nameComp->Data() << std::endl
        << std::string(spaces + 2, ' ')
        << "- Parent: " << entityInfo(_modelEntity, _ecm)
        << std::endl;

      const auto inertialComp = _ecm.Component<components::Inertial>(entity);

      if (inertialComp)
      {
        const auto inertialMatrix = inertialComp->Data().MassMatrix();
        const auto mass = inertialComp->Data().MassMatrix().Mass();

        std::cout << std::string(spaces + 2, ' ')
          << "- Mass (kg): " << std::to_string(mass) << std::endl
          << std::string(spaces + 2, ' ')
          << "- Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:"
          << std::endl
          << poseInfo(inertialComp->Data().Pose(), spaces + 4)
          << std::endl
          << std::string(spaces + 2, ' ')
          << "- Inertial Matrix (kg.m^2):\n"
          << std::string(spaces + 4, ' ') << "["
          << std::to_string(inertialMatrix.Ixx()) << " "
          << std::to_string(inertialMatrix.Ixy()) << " "
          << std::to_string(inertialMatrix.Ixz()) << "]\n"
          << std::string(spaces + 4, ' ') << "["
          << std::to_string(inertialMatrix.Ixy()) << " "
          << std::to_string(inertialMatrix.Iyy()) << " "
          << std::to_string(inertialMatrix.Iyz()) << "]\n"
          << std::string(spaces + 4, ' ') << "["
          << std::to_string(inertialMatrix.Ixz()) << " "
          << std::to_string(inertialMatrix.Iyz()) << " "
          << std::to_string(inertialMatrix.Izz()) << "]"
          << std::endl;
      }

      printPose(entity, _ecm, spaces + 2);

      spaces += 2;
    }

    const auto sensors = _ecm.EntitiesByComponents(
      components::ParentEntity(entity), components::Sensor());
    for (const auto &sensor : sensors)
    {
      const auto sensorNameComp = _ecm.Component<components::Name>(sensor);
      if (!_sensorName.empty() && _sensorName != sensorNameComp->Data())
        continue;

      std::cout << std::string(spaces, ' ')
        << "- Sensor [" << sensor << "]\n";
      std::cout << std::string(spaces + 2, ' ')
        << "- Name: " << sensorNameComp->Data() << "\n"
        << std::string(spaces + 2, ' ')
        << "- Parent: " << entityInfo(_modelEntity, _ecm) << std::endl;
      printPose(sensor, _ecm, spaces + 2);

      // Run through all the sensor print statements. Each function will
      // exit early if the the sensor is the wrong type.
      printAirPressure(sensor, _ecm, spaces + 2);
      printAltimeter(sensor, _ecm, spaces + 2);
      printCamera(sensor, _ecm, spaces + 2);
      printGpuLidar(sensor, _ecm, spaces + 2);
      printImu(sensor, _ecm, spaces + 2);
      printMagnetometer(sensor, _ecm, spaces + 2);
      printRgbdCamera(sensor, _ecm, spaces + 2);
    }
  }
}

//////////////////////////////////////////////////
// \brief Print the model joints information.
// \param[in] _modelEntity Entity of the model requested.
// \param[in] _ecm ECM ready for requests.
// \param[in] _jointName Joint to be printed, if nullptr, print all joints.
void printJoints(const uint64_t _modelEntity,
                const EntityComponentManager &_ecm,
                const std::string &_jointName,
                int _spaces)
{
  static const std::map<sdf::JointType, std::string> jointTypes =
  {
    {sdf::JointType::REVOLUTE, "revolute"},
    {sdf::JointType::BALL, "ball"},
    {sdf::JointType::CONTINUOUS, "continuous"},
    {sdf::JointType::FIXED, "fixed"},
    {sdf::JointType::GEARBOX, "gearbox"},
    {sdf::JointType::PRISMATIC,  "prismatic"},
    {sdf::JointType::REVOLUTE2, "revolute2"},
    {sdf::JointType::SCREW, "screw"},
    {sdf::JointType::UNIVERSAL, "universal"}
  };

  const auto joints = _ecm.EntitiesByComponents(
      components::ParentEntity(_modelEntity), components::Joint());

  for (const auto &entity : joints)
  {
    const auto nameComp = _ecm.Component<components::Name>(entity);

    if (_jointName.length() && _jointName != nameComp->Data())
      continue;

    int spaces = _spaces;
    std::cout << std::string(_spaces, ' ')
      << "- Joint [" << entity << "]" << std::endl;
    spaces += 2;

    std::cout << std::string(spaces, ' ')
      << "- Name: " << nameComp->Data() << std::endl
      << std::string(spaces, ' ')
      << "- Parent: " << entityInfo(_modelEntity, _ecm)
      << std::endl;

    const auto jointTypeComp = _ecm.Component<components::JointType>(entity);
    if (jointTypeComp)
    {
      std::cout << std::string(spaces, ' ')
        << "- Type: " << jointTypes.at(jointTypeComp->Data()) << std::endl;
    }

    const auto childLinkComp =
        _ecm.Component<components::ChildLinkName>(entity);
    const auto parentLinkComp =
        _ecm.Component<components::ParentLinkName>(entity);

    if (childLinkComp && parentLinkComp)
    {
      std::cout << std::string(spaces, ' ') << "- Parent Link: "
                << entityInfo(parentLinkComp->Data(), _ecm) << "\n"
                << std::string(spaces, ' ') << "- Child Link: "
                << entityInfo(childLinkComp->Data(), _ecm) << "\n";
    }

    const auto poseComp = _ecm.Component<components::Pose>(entity);
    if (poseComp)
    {
      std::cout << std::string(spaces, ' ')
        << "- Pose [ XYZ (m) ] [ RPY (rad) ]:" << std::endl
        << poseInfo(poseComp->Data(), spaces + 2) << std::endl;
    }

    const auto axisComp = _ecm.Component<components::JointAxis>(entity);
    if (axisComp)
    {
      std::cout << std::string(spaces, ' ') << "- Axis unit vector [ XYZ ]:\n"
        << std::string(spaces + 2, ' ') << "[" << axisComp->Data().Xyz()
        << "]\n";
    }
  }
}

//////////////////////////////////////////////////
extern "C" void cmdModelList()
{
  EntityComponentManager ecm{};
  if (!populateECM(ecm))
  {
    return;
  }

  auto world = ecm.EntityByComponents(components::World());
  if (kNullEntity == world)
  {
    std::cout << "No world found." << std::endl;
    return;
  }

  const auto models = ecm.EntitiesByComponents(
    components::ParentEntity(world), components::Model());

  if (models.size() == 0)
  {
    std::cout << "No models in world [" << world << "]" << std::endl;
    return;
  }

  std::cout << "Available models:" << std::endl;

  for (const auto &m : models)
  {
    const auto nameComp = ecm.Component<components::Name>(m);
    std::cout << "    - " << nameComp->Data() << std::endl;
  }
}

//////////////////////////////////////////////////
extern "C" void cmdModelInfo(
    const char *_modelName, int _pose, const char *_linkName,
    const char *_jointName, const char *_sensorName)
{
  std::string linkName{""};
  if (_linkName)
    linkName = _linkName;
  std::string jointName{""};
  if (_jointName)
    jointName = _jointName;
  std::string sensorName{""};
  if (_sensorName)
    sensorName = _sensorName;
  bool printAll{false};
  if (!_pose && !_linkName && !_jointName && !_sensorName)
    printAll = true;

  if (!_modelName)
  {
    std::cerr << std::endl << "Model name not found" << std::endl;
    return;
  }

  EntityComponentManager ecm{};
  if (!populateECM(ecm))
    return;

  // Get the desired model entity.
  auto entity = ecm.EntityByComponents(components::Name(_modelName),
      components::Model());

  if (entity == kNullEntity)
    std::cout << "No model named <" << _modelName << "> was found" << std::endl;

  int spaces = 0;
  // Get the pose of the model
  if (printAll || _pose)
  {
    printModelInfo(entity, ecm);
    spaces += 2;
  }

  // Get the links information
  if (printAll || _linkName != nullptr || _sensorName != nullptr)
    printLinks(entity, ecm, linkName, sensorName, spaces);

  // Get the joints information
  if (printAll || (_jointName != nullptr))
    printJoints(entity, ecm, jointName, spaces);
}
