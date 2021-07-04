/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include <string>
#include <vector>
#include <chrono>

#include "RealSensePlugin.hh"

#include <ignition/sensors/Manager.hh>

#include <ignition/rendering/DepthCamera.hh>
#include <ignition/rendering/Camera.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/common/Image.hh>
#include "ignition/gazebo/components/Name.hh"

#include <ignition/msgs.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <ignition/transport/Node.hh>

#define DEPTH_PUB_FREQ_HZ 60
#define COLOR_PUB_FREQ_HZ 60
#define IRED1_PUB_FREQ_HZ 60
#define IRED2_PUB_FREQ_HZ 60

#define DEPTH_CAMERA_NAME "depth"
#define COLOR_CAMERA_NAME "color"
#define IRED1_CAMERA_NAME "ired1"
#define IRED2_CAMERA_NAME "ired2"

#define DEPTH_CAMERA_TOPIC "depth"
#define COLOR_CAMERA_TOPIC "color"
#define IRED1_CAMERA_TOPIC "infrared"
#define IRED2_CAMERA_TOPIC "infrared2"

#define DEPTH_NEAR_CLIP_M 0.3
#define DEPTH_FAR_CLIP_M 10.0
#define DEPTH_SCALE_M 0.001

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::RealSensePluginPrivate
{
    /// \brief Callback that publishes a received Depth Camera Frame as an
    /// ImageStamped message.
    public: void OnNewDepthFrame();

    /// \brief Callback that publishes a received Camera Frame as an
    /// ImageStamped message.
    public: void OnNewFrame(const rendering::CameraPtr cam, const ignition::transport::Node::Publisher pub);
        
    /// \brief a model interface
    public: Model rsModel{kNullEntity};
  	
    /// \brief Pointer to the Depth Camera Renderer
    public: rendering::DepthCameraPtr depthCam;

    /// \brief Pointer to the Color Camera Renderer.
    public: rendering::CameraPtr colorCam;

    /// \brief Pointer to the Infrared Camera Renderer.
    public: rendering::CameraPtr ired1Cam;

    /// \brief Pointer to the Infrared2 Camera Renderer.
    public: rendering::CameraPtr ired2Cam;

    /// \brief Pointer to the transport Node.
    public: transport::Node transportNode;

    /// \brief Store Real Sense depth map data.
    public: std::vector<uint16_t> depthMap;

    /// \brief Pointer to the Depth Publisher.
    public: transport::Node::Publisher depthPub;

    /// \brief Pointer to the Color Publisher.
    public: transport::Node::Publisher colorPub;

    /// \brief Pointer to the Infrared Publisher.
    public: transport::Node::Publisher ired1Pub;		

    /// \brief Pointer to the Infrared2 Publisher.
    public: transport::Node::Publisher ired2Pub;

    /// \brief sensor manager
    public: sensors::Manager smanager;

    /// \brief String to hold camera prefix
    public: std::string prefix;

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene;

    /// \brief Pointer to an image to be published
    public: ignition::rendering::Image image;    
};
//////////////////////////////////////////////////////
RealSensePlugin::RealSensePlugin() : dataPtr(new RealSensePluginPrivate)
{
  this->dataPtr->depthCam = nullptr;
  this->dataPtr->ired1Cam = nullptr;
  this->dataPtr->ired2Cam = nullptr;
  this->dataPtr->colorCam = nullptr;
  this->dataPtr->prefix = "";
}

//////////////////////////////////////////////////////
RealSensePlugin::~RealSensePlugin()
{
}

//////////////////////////////////////////////////////
void RealSensePlugin::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)

{
  // Output the name of the model
  std::cout << std::endl
            << "RealSensePlugin: The realsense_camera plugin is attach to model "
            << this->dataPtr->rsModel.Name(_ecm) << std::endl;

  // model interface
  auto model = this->dataPtr->rsModel;

  // set camera prefix
  if (_sdf->HasElement("prefix")) {
    this->dataPtr->prefix = model.Name(_ecm) + "::" + scopedName(_entity, _ecm) + "::" + _sdf->Get<std::string>("prefix");
  } else {
    this->dataPtr->prefix = model.Name(_ecm) + "::" + scopedName(_entity, _ecm) + "::link::"; 
  }

  // Get Camera sensors
  this->dataPtr->depthCam = std::dynamic_pointer_cast<rendering::DepthCamera>(this->dataPtr->scene->SensorByName(this->dataPtr->prefix + std::string("depth")));

  this->dataPtr->ired1Cam = std::dynamic_pointer_cast<rendering::Camera>(this->dataPtr->scene->SensorByName(this->dataPtr->prefix + std::string("ired1")));

  this->dataPtr->ired2Cam = std::dynamic_pointer_cast<rendering::Camera>(this->dataPtr->scene->SensorByName(this->dataPtr->prefix + std::string("ireed2") ));

  this->dataPtr->colorCam = std::dynamic_pointer_cast<rendering::Camera>(this->dataPtr->scene->SensorByName(this->dataPtr->prefix + std::string("color")));


  if (!this->dataPtr->depthCam)
  {
    std::cerr << "RealSensePlugin: Depth Camera has not been found"
              << std::endl;
    return;
  }
  if (!this->dataPtr->ired1Cam)
  {
    std::cerr << "RealSensePlugin: InfraRed Camera 1 has not been found"
              << std::endl;
    return;
  }
  if (!this->dataPtr->ired2Cam)
  {
    std::cerr << "RealSensePlugin: InfraRed Camera 2 has not been found"
              << std::endl;
    return;
  }
  if (!this->dataPtr->colorCam)
  {
    std::cerr << "RealSensePlugin: Color Camera has not been found"
              << std::endl;
    return;
  }

  // Resize Depth Map dimensions
  try
  {
    this->dataPtr->depthMap.resize(this->dataPtr->depthCam->ImageWidth() * this->dataPtr->depthCam->ImageHeight());
  }
  catch (std::bad_alloc &e)
  {
    std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
              << std::endl;
    return;
  }

  // Setup Publishers
  std::string rsTopicRoot = "~/" + model.Name(_ecm) + "/rs/stream/";

  this->dataPtr->depthPub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + DEPTH_CAMERA_TOPIC);

  this->dataPtr->colorPub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + COLOR_CAMERA_TOPIC);

  this->dataPtr->ired1Pub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + IRED1_CAMERA_TOPIC);

  this->dataPtr->ired2Pub = this->dataPtr->transportNode.Advertise<msgs::Image>(rsTopicRoot + IRED2_CAMERA_TOPIC);  
 	  
}

//////////////////////////////////////////////////////
void RealSensePluginPrivate::OnNewFrame(rendering::CameraPtr cam, transport::Node::Publisher pub)

{
  IGN_PROFILE("RealSensePluginPrivate::OnNewFrame");

  ignition::msgs::Image msg;

  unsigned char *data = this->image.Data<unsigned char>();

  // Set Image Dimensions
  msg.set_width(cam->ImageWidth());
  msg.set_height(cam->ImageHeight());

  // Set Image Pixel Format 
  msg.set_pixel_format_type(ignition::msgs::PixelFormatType::RGB_INT8);

  // Set Image Data
  msg.set_step(cam->ImageWidth() * cam->ImageHeight());
  msg.set_data(data, cam->ImageHeight()*cam->ImageWidth());

  pub.Publish(msg);
}
//////////////////////////////////////////////////////

void RealSensePluginPrivate::OnNewDepthFrame()
{
  IGN_PROFILE("RealSensePluginPrivate::OnNewDepthFrame");
  // Get Depth Map dimensions
  unsigned int imageSize = this->depthCam->ImageWidth() *
                           this->depthCam->ImageHeight();

  // Check if depthMap size is equivalent to imageSize
  if (this->depthMap.size() != imageSize)
  {
    try
    {
      this->depthMap.resize(imageSize);
    }
    catch (std::bad_alloc &e)
    {
      std::cerr << "RealSensePlugin: depthMap allocation failed: " << e.what()
                << std::endl;
      return;
    }
  }

  // Instantiate message
  msgs::Image msg;

  // Convert Float depth data to RealSense depth data
  const float *depthDataFloat = this->depthCam->DepthData();

  for (unsigned int i = 0; i < imageSize; ++i)
  {
    // Check clipping and overflow
    if (depthDataFloat[i] < DEPTH_NEAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_FAR_CLIP_M ||
        depthDataFloat[i] > DEPTH_SCALE_M * UINT16_MAX ||
        depthDataFloat[i] < 0)
    {
      this->depthMap[i] = 0;
    }
    else
    {
      this->depthMap[i] = (uint16_t)(depthDataFloat[i] / DEPTH_SCALE_M);
    }
  }

  // Pack realsense scaled depth map
  msg.set_width(this->depthCam->ImageWidth());
  msg.set_height(this->depthCam->ImageHeight());
  msg.set_pixel_format_type(ignition::msgs::PixelFormatType::L_INT16);
  msg.set_step(this->depthCam->ImageWidth() * this->depthCam->ImageHeight());

  msg.set_data(this->depthMap.data(), sizeof(*this->depthMap.data() * imageSize));

  // Publish realsense scaled depth map
  this->depthPub.Publish(msg);

}

//////////////////////////////////////////////////////////
void RealSensePlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
      IGN_PROFILE("RealSensePlugin::PreUpdate");

	if (_info.dt < std::chrono::steady_clock::duration::zero())
  	{
      ignwarn << "Detected jump back in time ["
        	  << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
              << "s]. System may not work properly." << std::endl;
  	}
}

IGNITION_ADD_PLUGIN(RealSensePlugin,
  ignition::gazebo::System,
  RealSensePlugin::ISystemConfigure,
  RealSensePlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RealSensePlugin,"ignition::gazebo::systems::RealSensePlugin")
