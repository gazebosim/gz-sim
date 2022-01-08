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
#ifndef IGNITION_SENSORS_CustomRenderingSensor_HH_
#define IGNITION_SENSORS_CustomRenderingSensor_HH_

#include <cstdint>
#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include <ignition/common/SuppressWarning.hh>

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4005)
#pragma warning(disable: 4251)
#endif
#include <ignition/msgs.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#ifdef _WIN32
#pragma warning(push)
#pragma warning(disable: 4251)
#endif
#include <ignition/rendering/Camera.hh>
#ifdef _WIN32
#pragma warning(pop)
#endif

#include "ignition/sensors/camera/Export.hh"
#include "ignition/sensors/config.hh"
#include "ignition/sensors/Export.hh"
#include "ignition/sensors/RenderingSensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class CustomRenderingSensorPrivate;

    /// \brief Custom Camera Sensor Class
    class IGNITION_SENSORS_CAMERA_VISIBLE CustomRenderingSensor : public RenderingSensor
    {
      /// \brief constructor
      public: CustomRenderingSensor();

      /// \brief destructor
      public: virtual ~CustomRenderingSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Force the sensor to generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the rendering scene.
      /// \param[in] _scene Pointer to the scene
      public: virtual void SetScene(
                  ignition::rendering::ScenePtr _scene) override;

      /// \brief Get image width.
      /// \return width of the image
      public: virtual unsigned int ImageWidth() const;

      /// \brief Get image height.
      /// \return height of the image
      public: virtual unsigned int ImageHeight() const;

      /// \brief Topic where camera info is published.
      /// \return Camera info topic.
      public: std::string InfoTopic() const;

      /// \brief Set baseline for stereo cameras. This is used to populate the
      /// projection matrix in the camera info message.
      /// \param[in] _baseline The distance from the 1st camera, in meters.
      public: void SetBaseline(double _baseline);

      /// \brief Get baseline for stereo cameras.
      /// \return The distance from the 1st camera, in meters.
      public: double Baseline() const;

      /// \brief Advertise camera info topic.
      /// \return True if successful.
      protected: bool AdvertiseInfo();

      /// \brief Advertise camera info topic.
      /// This version takes a string that allows one to override the
      /// camera_info topic.
      /// \param[in] _topic The topic on which camera info is to be published.
      /// \return True if successful.
      protected: bool AdvertiseInfo(const std::string &_topic);

      /// \brief Populate camera info message.
      /// \param[in] _cameraSdf Pointer to SDF object containing camera
      /// information.
      protected: void PopulateInfo(const sdf::Camera *_cameraSdf);

      /// \brief Publish camera info message.
      /// \param[in] _now The current time
      protected: void PublishInfo(
        const std::chrono::steady_clock::duration &_now);

      /// \brief Create a camera in a scene
      /// \return True on success.
      private: bool CreateCamera();

      /// \brief Callback that is triggered when the scene changes on
      /// the Manager.
      /// \param[in] _scene Pointer to the new scene.
      private: void OnSceneChange(ignition::rendering::ScenePtr /*_scene*/);

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<CustomRenderingSensorPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif
