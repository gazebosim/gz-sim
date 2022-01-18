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
#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs/camera_info.pb.h>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/common/Event.hh>
#include <ignition/common/Image.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/common/StringUtils.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/transport/Node.hh>

#include "CustomRenderingSensor.hh"
#include "ignition/sensors/Manager.hh"
#include "ignition/sensors/RenderingEvents.hh"
#include "ignition/sensors/SensorFactory.hh"
#include "ignition/sensors/SensorTypes.hh"

using namespace ignition;
using namespace sensors;

/// \brief Private data for CustomRenderingSensor
class ignition::sensors::CustomRenderingSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub_left;
  public: transport::Node::Publisher pub_right;

  /// \brief Camera info publisher to publish images
  public: transport::Node::Publisher infoPub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: ignition::rendering::CameraPtr camera_left;
  public: ignition::rendering::CameraPtr camera_right;

  /// \brief Pointer to an image to be published
  public: ignition::rendering::Image image_left;
  public: ignition::rendering::Image image_right;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: ignition::common::EventT<
          void(const ignition::msgs::Image &)> imageEvent;

  /// \brief Connection to the Manager's scene change event.
  public: ignition::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief Camera information message.
  public: msgs::CameraInfo infoMsg;

  /// \brief Topic for info message.
  public: std::string infoTopic{""};

  /// \brief Baseline for stereo cameras.
  public: double baseline{0.0};

  /// \brief Flag to indicate if sensor is generating data
  public: bool generatingData = false;

  public: sdf::Camera temp_cameraSdf;
  public: ignition::math::Pose3d left_camera_pose;

  public: ignition::math::Pose3d right_camera_pose;

};

//////////////////////////////////////////////////
bool CustomRenderingSensor::CreateCamera()
{
  auto cameraSdf = &(this->dataPtr->temp_cameraSdf);

  if (!cameraSdf)
  {
    ignerr << "Unable to access camera SDF element.\n";
    return false;
  }

  this->PopulateInfo(cameraSdf);

  auto cameraSdfElement = cameraSdf->Element();
  this->dataPtr->left_camera_pose = cameraSdfElement->Get<ignition::math::Pose3d>("vs:left_camera_pose");
  this->dataPtr->right_camera_pose = cameraSdfElement->Get<ignition::math::Pose3d>("vs:right_camera_pose");

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  // To prevent sensor not found error in SceneManager
  auto temp_camera_object = this->Scene()->CreateCamera(this->Name());

  // Left and right cameras
  this->dataPtr->camera_left = this->Scene()->CreateCamera("camera_left_sensor");
  this->dataPtr->camera_left->SetImageWidth(width);
  this->dataPtr->camera_left->SetImageHeight(height);
  this->dataPtr->camera_left->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera_left->SetFarClipPlane(cameraSdf->FarClip());
  this->dataPtr->camera_left->SetVisibilityMask(cameraSdf->VisibilityMask());
  this->AddSensor(this->dataPtr->camera_left);

  this->dataPtr->camera_right = this->Scene()->CreateCamera("camera_right_sensor");
  this->dataPtr->camera_right->SetImageWidth(width);
  this->dataPtr->camera_right->SetImageHeight(height);
  this->dataPtr->camera_right->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera_right->SetFarClipPlane(cameraSdf->FarClip());
  this->dataPtr->camera_right->SetVisibilityMask(cameraSdf->VisibilityMask());
  this->AddSensor(this->dataPtr->camera_right);

  this->dataPtr->camera_left->SetAntiAliasing(2);
  this->dataPtr->camera_right->SetAntiAliasing(2);

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.01 || angle > IGN_PI*2)
  {
    ignerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }
  this->dataPtr->camera_left->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera_left->SetHFOV(angle);
  this->dataPtr->camera_right->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera_right->SetHFOV(angle);

  sdf::PixelFormatType pixelFormat = cameraSdf->PixelFormat();
  switch (pixelFormat)
  {
    case sdf::PixelFormatType::RGB_INT8:
      this->dataPtr->camera_left->SetImageFormat(ignition::rendering::PF_R8G8B8);
      this->dataPtr->camera_right->SetImageFormat(ignition::rendering::PF_R8G8B8);
      break;
    default:
      ignerr << "Unsupported pixel format ["
        << static_cast<int>(pixelFormat) << "]\n";
      break;
  }

  this->dataPtr->image_left = this->dataPtr->camera_left->CreateImage();
  this->dataPtr->image_right = this->dataPtr->camera_right->CreateImage();

  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera_left);
  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera_right);

  return true;
}

//////////////////////////////////////////////////
CustomRenderingSensor::CustomRenderingSensor()
  : dataPtr(new CustomRenderingSensorPrivate())
{
}

//////////////////////////////////////////////////
CustomRenderingSensor::~CustomRenderingSensor()
{
}

//////////////////////////////////////////////////
bool CustomRenderingSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool CustomRenderingSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }
  
  auto sensor_element = _sdf.Element();
  auto cam_element = sensor_element->GetElement("camera");
  this->dataPtr->temp_cameraSdf.Load(cam_element);

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::CUSTOM)
  {
    ignerr << "Attempting to a load a custom camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  this->dataPtr->sdfSensor = _sdf;

  if (this->Topic().empty())
    this->SetTopic("/camera_left");
  
  this->dataPtr->pub_left =
      this->dataPtr->node.Advertise<ignition::msgs::Image>("/camera_left");
  this->dataPtr->pub_right =
      this->dataPtr->node.Advertise<ignition::msgs::Image>("/camera_right");

  if (!this->dataPtr->pub_left ||
      !this->dataPtr->pub_right)
  {
    ignerr << "Unable to create publishers on topics" <<std::endl;
    return false;
  }

  if (!this->AdvertiseInfo())
    return false;

  if (this->Scene())
    this->CreateCamera();

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&CustomRenderingSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool CustomRenderingSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
void CustomRenderingSensor::SetScene(ignition::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->dataPtr->camera_left = nullptr;
    this->dataPtr->camera_right = nullptr;
    RenderingSensor::SetScene(_scene);
    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

//////////////////////////////////////////////////
bool CustomRenderingSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  IGN_PROFILE("CustomRenderingSensor::Update");
  if (!this->dataPtr->initialized)
  {
    ignerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->camera_left || !this->dataPtr->camera_right)
  {
    ignerr << "Camera doesn't exist.\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // move the camera to the current pose
  this->dataPtr->camera_right->SetLocalPose(this->dataPtr->right_camera_pose);
  this->dataPtr->camera_left->SetLocalPose(this->dataPtr->left_camera_pose);

  this->dataPtr->generatingData = true;

  // generate sensor data
  this->Render();
  {
    IGN_PROFILE("CustomRenderingSensor::Update Copy image");
    this->dataPtr->camera_left->Copy(this->dataPtr->image_left);
    this->dataPtr->camera_right->Copy(this->dataPtr->image_right);
  }

  unsigned int width = this->dataPtr->camera_left->ImageWidth();
  unsigned int height = this->dataPtr->camera_left->ImageHeight();
  unsigned char *data_left = this->dataPtr->image_left.Data<unsigned char>();
  unsigned char *data_right = this->dataPtr->image_right.Data<unsigned char>();

  msgs::PixelFormatType msgsPixelFormat =
    msgs::PixelFormatType::UNKNOWN_PIXEL_FORMAT;

  switch (this->dataPtr->camera_left->ImageFormat())
  {
    case ignition::rendering::PF_R8G8B8:
      msgsPixelFormat = msgs::PixelFormatType::RGB_INT8;
      break;
    default:
      ignerr << "Unsupported pixel format ["
        << this->dataPtr->camera_left->ImageFormat() << "]\n";
      break;
  }

  // create message -- left and right cam
  ignition::msgs::Image msg_left;
  {
    IGN_PROFILE("CustomRenderingSensor::Update Message");
    msg_left.set_width(width);
    msg_left.set_height(height);
    msg_left.set_step(width * rendering::PixelUtil::BytesPerPixel(
                 this->dataPtr->camera_left->ImageFormat()));
    msg_left.set_pixel_format_type(msgsPixelFormat);
    *msg_left.mutable_header()->mutable_stamp() = msgs::Convert(_now);
    auto frame = msg_left.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    msg_left.set_data(data_left, this->dataPtr->camera_left->ImageMemorySize());
  }
  ignition::msgs::Image msg_right;
  {
    IGN_PROFILE("CustomRenderingSensor::Update Message");
    msg_right.set_width(width);
    msg_right.set_height(height);
    msg_right.set_step(width * rendering::PixelUtil::BytesPerPixel(
                 this->dataPtr->camera_right->ImageFormat()));
    msg_right.set_pixel_format_type(msgsPixelFormat);
    *msg_right.mutable_header()->mutable_stamp() = msgs::Convert(_now);
    auto frame = msg_right.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->Name());
    msg_right.set_data(data_right, this->dataPtr->camera_right->ImageMemorySize());
  }

  // publish the image message
  {
    this->AddSequence(msg_left.mutable_header());
    IGN_PROFILE("CustomRenderingSensor::Update Publish");
    this->dataPtr->pub_left.Publish(msg_left);
    this->dataPtr->pub_right.Publish(msg_right);

    // publish the camera info message
    this->PublishInfo(_now);
  }

  return true;
}

//////////////////////////////////////////////////
unsigned int CustomRenderingSensor::ImageWidth() const
{
  if (this->dataPtr->camera_left && this->dataPtr->camera_right)
    return this->dataPtr->camera_left->ImageWidth();
  return 0;
}

//////////////////////////////////////////////////
unsigned int CustomRenderingSensor::ImageHeight() const
{
  if (this->dataPtr->camera_left && this->dataPtr->camera_right)
    return this->dataPtr->camera_left->ImageHeight();
  return 0;
}

//////////////////////////////////////////////////
std::string CustomRenderingSensor::InfoTopic() const
{
  return this->dataPtr->infoTopic;
}

//////////////////////////////////////////////////
bool CustomRenderingSensor::AdvertiseInfo()
{
  auto parts = common::Split(this->Topic(), '/');
  parts.pop_back();

  for (const auto &part : parts)
  {
    if (!part.empty())
      this->dataPtr->infoTopic += "/" + part;
  }
  this->dataPtr->infoTopic += "/camera_info";

  return this->AdvertiseInfo(this->dataPtr->infoTopic);
}

//////////////////////////////////////////////////
bool CustomRenderingSensor::AdvertiseInfo(const std::string &_topic)
{
  this->dataPtr->infoTopic = _topic;

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<ignition::msgs::CameraInfo>(
      this->dataPtr->infoTopic);
  if (!this->dataPtr->infoPub)
  {
    ignerr << "Unable to create publisher on topic ["
      << this->dataPtr->infoTopic << "].\n";
  }
  else
  {
    igndbg << "Camera info for [" << this->Name() << "] advertised on ["
           << this->dataPtr->infoTopic << "]" << std::endl;
  }

  return this->dataPtr->infoPub;
}

//////////////////////////////////////////////////
void CustomRenderingSensor::PublishInfo(
  const std::chrono::steady_clock::duration &_now)
{
  *this->dataPtr->infoMsg.mutable_header()->mutable_stamp() =
    msgs::Convert(_now);
  this->dataPtr->infoPub.Publish(this->dataPtr->infoMsg);
}

//////////////////////////////////////////////////
void CustomRenderingSensor::PopulateInfo(const sdf::Camera *_cameraSdf)
{
  unsigned int width = _cameraSdf->ImageWidth();
  unsigned int height = _cameraSdf->ImageHeight();

  msgs::CameraInfo::Distortion *distortion =
    this->dataPtr->infoMsg.mutable_distortion();

  distortion->set_model(msgs::CameraInfo::Distortion::PLUMB_BOB);
  distortion->add_k(_cameraSdf->DistortionK1());
  distortion->add_k(_cameraSdf->DistortionK2());
  distortion->add_k(_cameraSdf->DistortionP1());
  distortion->add_k(_cameraSdf->DistortionP2());
  distortion->add_k(_cameraSdf->DistortionK3());

  msgs::CameraInfo::Intrinsics *intrinsics =
    this->dataPtr->infoMsg.mutable_intrinsics();

  intrinsics->add_k(_cameraSdf->LensIntrinsicsFx());
  intrinsics->add_k(0.0);
  intrinsics->add_k(_cameraSdf->LensIntrinsicsCx());

  intrinsics->add_k(0.0);
  intrinsics->add_k(_cameraSdf->LensIntrinsicsFy());
  intrinsics->add_k(_cameraSdf->LensIntrinsicsCy());

  intrinsics->add_k(0.0);
  intrinsics->add_k(0.0);
  intrinsics->add_k(1.0);

  // TODO(anyone) Get tx and ty from SDF
  msgs::CameraInfo::Projection *proj =
    this->dataPtr->infoMsg.mutable_projection();

  proj->add_p(_cameraSdf->LensIntrinsicsFx());
  proj->add_p(0.0);
  proj->add_p(_cameraSdf->LensIntrinsicsCx());
  proj->add_p(-_cameraSdf->LensIntrinsicsFx() * this->dataPtr->baseline);

  proj->add_p(0.0);
  proj->add_p(_cameraSdf->LensIntrinsicsFy());
  proj->add_p(_cameraSdf->LensIntrinsicsCy());
  proj->add_p(0.0);

  proj->add_p(0.0);
  proj->add_p(0.0);
  proj->add_p(1.0);
  proj->add_p(0.0);

  // Set the rectification matrix to identity
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);

  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);

  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);

  auto infoFrame = this->dataPtr->infoMsg.mutable_header()->add_data();
  infoFrame->set_key("frame_id");
  infoFrame->add_value(this->Name());

  this->dataPtr->infoMsg.set_width(width);
  this->dataPtr->infoMsg.set_height(height);
}
