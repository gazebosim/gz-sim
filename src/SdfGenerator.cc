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

#include "SdfGenerator.hh"

#include <ctype.h>
#include <memory>
#include <vector>

#include <sdf/sdf.hh>

#include <gz/common/URI.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/AirPressureSensor.hh"
#include "gz/sim/components/AirSpeedSensor.hh"
#include "gz/sim/components/Altimeter.hh"
#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/ContactSensor.hh"
#include "gz/sim/components/DepthCamera.hh"
#include "gz/sim/components/ForceTorque.hh"
#include "gz/sim/components/GpuLidar.hh"
#include "gz/sim/components/Imu.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/LogicalCamera.hh"
#include "gz/sim/components/Magnetometer.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/RgbdCamera.hh"
#include "gz/sim/components/SelfCollide.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/components/SegmentationCamera.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/ThreadPitch.hh"
#include "gz/sim/components/WindMode.hh"
#include "gz/sim/components/World.hh"


namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE
{
namespace sdf_generator
{
  /////////////////////////////////////////////////
  /// \brief Copy sdf::Element from component
  /// \param[in] _comp Component containing an sdf::Element. The component has
  /// to be a DOM object
  /// \param[out] _elem Output sdf::Element
  /// \returns False if the component is nullptr
  template <typename ComponentT>
  static bool copySdf(ComponentT *_comp, const sdf::ElementPtr &_elem)
  {
    if (nullptr != _comp)
    {
      _elem->Copy(_comp->Data().Element());
      return true;
    }
    return false;
  }

  /////////////////////////////////////////////////
  /// \brief Remove version number from Fuel URI
  /// \param[in, out] _uri The URI from which the version number is removed.
  /// This is assumed to be a Fuel URI where, if there is a version number, it's
  /// the last part of the URI.
  static void removeVersionFromUri(common::URI &_uri)
  {
    auto uriSplit = common::split(common::trimmed(_uri.Path().Str()), "/");
    if (uriSplit.size() > 0)
    {
      try
      {
        // This assumes that model names cannot be purely numerical.
        auto version = std::stol(uriSplit.back());
        // We can ignore the returned value since an exception will be thrown if
        // the conversion failed.
        static_cast<void>(version);
        uriSplit.pop_back();
        common::URIPath newPath;
        for (const auto &segment : uriSplit)
        {
          newPath /= segment;
        }
        _uri.Path() = newPath;
      }
      catch(const std::invalid_argument &)
      {
      }
    }
  }

  /////////////////////////////////////////////////
  /// \brief Determine if a model is inlined or included based on the file paths
  /// of the model and the world.
  /// \param[in] _modelDir Directory containing model
  /// \param[in] _worldDir Directory containing world
  /// \returns True if the source of the model is a separate file that was
  /// included into the world using the `<include>` tag.
  static bool isModelFromInclude(const std::string &_modelDir,
                                 const std::string &_worldDir)
  {
    // There are several cases to consider here
    // modelDir == "" , worldDir == ""
    //  - This is the case where the world and the model were loaded from
    //  a string, so the modelFromInclude = false
    // modelDir == "/some/path", worldDir == ""
    //  - The world was loaded from a string, but the model was included,
    //  so modelFromInclude = true
    // modelDir == "", worldDir == "/some/path"
    //  - The world was loaded from a file, but the model was loaded from
    //  a string (via UserCommands, for example), so modelFromInclude =
    //  false
    // modelDir == "/some/path", worldDir == "/some/path"
    //  - Both world and model were loaded from file. There two subcases
    //    - modelDir == worldDir
    //      - Model was inlined in the world file, so modelFromInclude =
    //      false
    //    - modelDir != worldDir
    //      - Model was loaded from a different file, so modelFromInclude
    //      = true
    //
    if (!_modelDir.empty() && _worldDir.empty())
    {
      return true;
    }
    else if (!_modelDir.empty() && !_worldDir.empty() &&
        (_modelDir != _worldDir))
    {
      return true;
    }

    return false;
  }

  /////////////////////////////////////////////////
  /// \brief Merge a configuration with another while making sure any parameters
  /// set in `_override` are copied as is.
  ///
  /// The idea here is that the initial configuration is a global configuration
  /// that can be overriden on a per model basis. The following snippet
  /// demonstrates the intent, even though it won't compile
  /// \code
  ///   EntityGeneratorConfig initialConfig;
  ///   initialConfig.expand_include_tags = true
  ///   initialConfig.save_fuel_model_version = true
  ///
  ///   EntityGeneratorConfig overrideConfig;
  ///   overrideConfig.expand_include_tags = false;
  ///
  ///   EntityGeneratorConfig combinedConfig = initialConfig;
  ///   mergeWithOverride(combinedConfig, overrideConfig);
  /// \endcode
  ///
  /// The contents of each of the configs is now:
  /// \code
  ///   initialConfig = {
  ///     expand_include_tags = true
  ///     save_fuel_model_version = true
  ///   }
  ///   overrideConfig = {
  ///     expand_include_tags = false
  ///     save_fuel_model_version = false (unset, but defaults to false)
  ///   }
  ///
  ///   combinedConfig = {
  ///     expand_include_tags = false
  ///     save_fuel_model_version = true
  ///   }
  /// \endcode
  ///
  /// This function is needed because neither `Message::CopyFrom` nor
  /// `Message::MergeFrom` do what we want. CopyFrom overwrites everything even
  /// if none of the parameters in overrideConfig are set. MergeFrom gets close,
  /// but doesn't overwrite if the parameter in `_overrideConfig` is set to
  /// false.
  ///
  /// \param[in, out] _initialConfig Initial configuration
  /// \param[in] _override Override configuration
  static void mergeWithOverride(
      msgs::SdfGeneratorConfig::EntityGeneratorConfig &_initialConfig,
      const msgs::SdfGeneratorConfig::EntityGeneratorConfig &_overrideConfig)
  {
    auto initialDesc = _initialConfig.GetDescriptor();
    auto initialRefl = _initialConfig.GetReflection();
    auto overrideDesc = _overrideConfig.GetDescriptor();
    auto overrideRefl = _overrideConfig.GetReflection();

    for (int i = 0; i < overrideDesc->field_count(); ++i)
    {
      // If a field is set in _overrideConfig, copy it over to initialConfig
      // overwriting anything there
      if (overrideRefl->HasField(_overrideConfig, overrideDesc->field(i)))
      {
        initialRefl->MutableMessage(&_initialConfig, initialDesc->field(i))
            ->CopyFrom(overrideRefl->GetMessage(
                _overrideConfig, overrideDesc->field(i)));
      }
    }
  }

  /////////////////////////////////////////////////
  /// \brief Recursively go through the child elements of the input element and
  /// update all relative URIs to absolute.
  ///
  /// URIs with http / https scheme won't be modified.
  ///
  /// For all other URIs, the resulting URI will have a "file://" scheme
  /// regardless of whether the original URI had the scheme. i.e, absolute URIs
  /// without the "file://" scheme will also be updated by this function.
  /// \param[in] _elem Input element to update
  /// \param[in] _prefixPath Path to be prepended to relative URIs.
  static void relativeToAbsoluteUri(const sdf::ElementPtr &_elem,
                                    const std::string &_prefixPath)
  {
    if (_elem->HasElement("uri"))
    {
      auto uriElem = _elem->GetElement("uri");
      auto uriStr = uriElem->Get<std::string>();
      // If the URI starts with "file://", it is assumed to be an
      // absolute path, so there is no need to update it.
      if (uriStr.find("file://") == std::string::npos &&
          uriStr.find("http://") == std::string::npos &&
          uriStr.find("https://") == std::string::npos)
      {
        if (uriStr[0] != '/')
        {
          // relative uri
          uriStr = common::joinPaths(_prefixPath, uriStr);
        }
        uriStr = std::string("file://") + uriStr;
        uriElem->Set(uriStr);
      }
    }
    else
    {
      for (auto child = _elem->GetFirstElement(); child;
           child = child->GetNextElement())
      {
        relativeToAbsoluteUri(child, _prefixPath);
      }
    }
  }

  /////////////////////////////////////////////////
  std::optional<std::string> generateWorld(
      const EntityComponentManager &_ecm, const Entity &_entity,
      const IncludeUriMap &_includeUriMap,
      const msgs::SdfGeneratorConfig &_config)
  {
    sdf::ElementPtr elem = std::make_shared<sdf::Element>();
    sdf::initFile("root.sdf", elem);
    auto worldElem = elem->AddElement("world");
    if (!updateWorldElement(worldElem, _ecm, _entity, _includeUriMap, _config))
      return std::nullopt;

    return elem->ToString("");
  }

  /////////////////////////////////////////////////
  bool updateWorldElement(sdf::ElementPtr _elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity,
                          const IncludeUriMap &_includeUriMap,
                          const msgs::SdfGeneratorConfig &_config)
  {
    const auto *worldSdf = _ecm.Component<components::WorldSdf>(_entity);

    if (nullptr == worldSdf)
      return false;

    if (!copySdf(_ecm.Component<components::WorldSdf>(_entity), _elem))
      return false;

    // First remove child entities of <world> whose names can be changed during
    // simulation (eg. models). Then we add them back from the data in the
    // ECM.
    // TODO(addisu) Remove actors
    std::vector<sdf::ElementPtr> toRemove;
    if (_elem->HasElement("model"))
    {
      for (auto modelElem = _elem->GetElement("model"); modelElem;
           modelElem = modelElem->GetNextElement("model"))
      {
        toRemove.push_back(modelElem);
      }
    }
    if (_elem->HasElement("light"))
    {
      for (auto lightElem = _elem->GetElement("light"); lightElem;
           lightElem = lightElem->GetNextElement("light"))
      {
        toRemove.push_back(lightElem);
      }
    }

    for (const auto &e : toRemove)
    {
      _elem->RemoveChild(e);
    }

    auto worldDir = common::parentPath(worldSdf->Data().Element()->FilePath());

    // models
    _ecm.Each<components::Model, components::ModelSdf>(
        [&](const Entity &_modelEntity, const components::Model *,
            const components::ModelSdf *_modelSdf)
        {
          // skip nested models as they are not direct children of world
          auto parentComp = _ecm.Component<components::ParentEntity>(
              _modelEntity);
          if (parentComp && parentComp->Data() != _entity)
            return true;

          auto modelDir =
              common::parentPath(_modelSdf->Data().Element()->FilePath());

          const std::string modelName =
              scopedName(_modelEntity, _ecm, "::", false);

          bool modelFromInclude = isModelFromInclude(modelDir, worldDir);

          auto uriMapIt = _includeUriMap.find(modelDir);

          auto modelConfig = _config.global_entity_gen_config();
          auto modelConfigIt =
              _config.override_entity_gen_configs().find(modelName);
          if (modelConfigIt != _config.override_entity_gen_configs().end())
          {
            mergeWithOverride(modelConfig, modelConfigIt->second);
          }

          if (modelConfig.expand_include_tags().data() || !modelFromInclude)
          {
            auto modelElem = _elem->AddElement("model");
            updateModelElement(modelElem, _ecm, _modelEntity);

            // Check & update possible //model/include(s)
            if (!modelConfig.expand_include_tags().data())
            {
              updateModelElementWithNestedInclude(modelElem,
                    modelConfig.save_fuel_version().data(), _includeUriMap);
            }
          }
          else if (uriMapIt != _includeUriMap.end())
          {
            // The fuel URI might have a version number. If it does, we remove
            // it unless saveFuelModelVersion is set to true.
            // Check if this is a fuel URI. We assume that it is a fuel URI if
            // the scheme is http or https.
            common::URI uri(uriMapIt->second);
            if (uri.Scheme() == "http" || uri.Scheme() == "https")
            {
              removeVersionFromUri(uri);
            }

            if (modelConfig.save_fuel_version().data())
            {
              // Find out the model version from the file path. Note that we
              // do this from the file path instead of the Fuel URI because the
              // URI may not contain version information.
              //
              // We are assuming here that, for Fuel models, the directory
              // containing the sdf file has the same name as the model version.
              // For example, if the uri is
              // https://example.org/1.0/test/models/Backpack
              // the path to the directory containing the sdf file (modelDir)
              // will be:
              // $HOME/.gz/fuel/example.org/test/models/Backpack/2/
              // and the basename of the directory is "1", which is the model
              // version.
              //
              // However, if symlinks (or other types of indirection) are used,
              // the pattern of modelDir will be different. The assumption here
              // is that regardless of the indirection, the name of the
              // directory containing the sdf file can be used as the version
              // number
              //
              uri.Path() /= common::basename(modelDir);
            }

            auto includeElem = _elem->AddElement("include");
            updateIncludeElement(includeElem, _ecm, _modelEntity, uri.Str());
          }
          else
          {
            // The model is not in the includeUriMap, but expandIncludeTags =
            // false, so we will assume that its uri is the file path of the
            // model on the local machine
            auto includeElem = _elem->AddElement("include");
            const std::string uri = "file://" + modelDir;
            updateIncludeElement(includeElem, _ecm, _modelEntity, uri);
          }
          return true;
        });

    // lights
    _ecm.Each<components::Light, components::ParentEntity>(
        [&](const Entity &_lightEntity,
            const components::Light *,
            const components::ParentEntity *_parent) -> bool
        {
          if (_parent->Data() != _entity)
            return true;

           auto lightElem = _elem->AddElement("light");
           updateLightElement(lightElem, _ecm, _lightEntity);

          return true;
        });

    return true;
  }

  /////////////////////////////////////////////////
  bool updateModelElement(const sdf::ElementPtr &_elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity)
  {
    if (!copySdf(_ecm.Component<components::ModelSdf>(_entity), _elem))
      return false;

    // Update sdf based on current components. Here are the list of components
    // to be updated:
    // - Name
    // - Pose
    // - Static
    // - SelfCollide
    // This list is to be updated as other components become updateable during
    // simulation
    auto *nameComp = _ecm.Component<components::Name>(_entity);
    _elem->GetAttribute("name")->Set(nameComp->Data());

    auto *poseComp = _ecm.Component<components::Pose>(_entity);

    auto poseElem = _elem->GetElement("pose");

    // Remove all attributes of poseElem
    for (const auto *attrName : {"relative_to", "degrees", "rotation_format"})
    {
      sdf::ParamPtr attr = poseElem->GetAttribute(attrName);
      if (nullptr != attr)
      {
        attr->Reset();
      }
    }
    poseElem->Set(poseComp->Data());

    // static
    auto *staticComp = _ecm.Component<components::Static>(_entity);
    if (staticComp)
      _elem->GetElement("static")->Set<bool>(staticComp->Data());

    // self collide
    auto *selfCollideComp = _ecm.Component<components::SelfCollide>(_entity);
    if (selfCollideComp)
      _elem->GetElement("self_collide")->Set<bool>(selfCollideComp->Data());

    const auto *pathComp =
      _ecm.Component<components::SourceFilePath>(_entity);

    if (_elem->HasElement("link"))
    {
      if (nullptr != pathComp)
      {
        // Update relative URIs to use absolute paths. Relative URIs work fine
        // in included models, but they have to be converted to absolute URIs
        // when the included model is expanded.
        relativeToAbsoluteUri(_elem, common::parentPath(pathComp->Data()));
      }

      // update links
      sdf::ElementPtr linkElem = _elem->GetElement("link");
      while (linkElem)
      {
        std::string linkName = linkElem->Get<std::string>("name");
        auto linkEnt = _ecm.EntityByComponents(
            components::ParentEntity(_entity), components::Name(linkName));
        if (linkEnt != kNullEntity)
          updateLinkElement(linkElem, _ecm, linkEnt);
        linkElem = linkElem->GetNextElement("link");
      }
    }

    if (_elem->HasElement("joint"))
    {
      // update joints
      sdf::ElementPtr jointElem = _elem->GetElement("joint");
      while (jointElem)
      {
        std::string jointName = jointElem->Get<std::string>("name");
        auto jointEnt = _ecm.EntityByComponents(
            components::ParentEntity(_entity), components::Name(jointName));
        if (jointEnt != kNullEntity)
          updateJointElement(jointElem, _ecm, jointEnt);
        jointElem = jointElem->GetNextElement("joint");
      }
    }

    return true;
  }

  /////////////////////////////////////////////////
  bool updateLinkElement(const sdf::ElementPtr &_elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity)
  {
    // Update sdf based on current components. Here are the list of components
    // to be updated:
    // - Name
    // - Pose
    // - Inertial
    // - WindMode
    // This list is to be updated as other components become updateable during
    // simulation
    auto *nameComp = _ecm.Component<components::Name>(_entity);
    _elem->GetAttribute("name")->Set(nameComp->Data());

    auto *poseComp = _ecm.Component<components::Pose>(_entity);

    auto poseElem = _elem->GetElement("pose");

    // Remove all attributes of poseElem
    for (const auto *attrName : {"relative_to", "degrees", "rotation_format"})
    {
      sdf::ParamPtr attr = poseElem->GetAttribute(attrName);
      if (nullptr != attr)
      {
        attr->Reset();
      }
    }
    poseElem->Set(poseComp->Data());

    // inertial
    auto inertialComp = _ecm.Component<components::Inertial>(_entity);
    if (inertialComp)
    {
      math::Inertiald inertial = inertialComp->Data();
      sdf::ElementPtr inertialElem = _elem->GetElement("inertial");
      inertialElem->GetElement("pose")->Set<math::Pose3d>(inertial.Pose());
      const math::MassMatrix3d &massMatrix = inertial.MassMatrix();
      inertialElem->GetElement("mass")->Set<double>(massMatrix.Mass());
      sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
      inertiaElem->GetElement("ixx")->Set<double>(massMatrix.Ixx());
      inertiaElem->GetElement("ixy")->Set<double>(massMatrix.Ixy());
      inertiaElem->GetElement("ixz")->Set<double>(massMatrix.Ixz());
      inertiaElem->GetElement("iyy")->Set<double>(massMatrix.Iyy());
      inertiaElem->GetElement("iyz")->Set<double>(massMatrix.Iyz());
      inertiaElem->GetElement("izz")->Set<double>(massMatrix.Izz());
    }

    // wind mode
    auto windModeComp = _ecm.Component<components::WindMode>(_entity);
    if (windModeComp)
    {
      bool windMode = windModeComp->Data();
      _elem->GetElement("enable_wind")->Set<bool>(windMode);
    }

    // update sensors
    if (_elem->HasElement("sensor"))
    {
      sdf::ElementPtr sensorElem = _elem->GetElement("sensor");
      while (sensorElem)
      {
        std::string sensorName = sensorElem->Get<std::string>("name");
        auto sensorEnt = _ecm.EntityByComponents(
            components::ParentEntity(_entity), components::Name(sensorName));
        if (sensorEnt != kNullEntity)
          updateSensorElement(sensorElem, _ecm, sensorEnt);
        sensorElem = sensorElem->GetNextElement("sensor");
      }
    }

    // update lights
    if (_elem->HasElement("light"))
    {
      sdf::ElementPtr lightElem = _elem->GetElement("light");
      while (lightElem)
      {
        std::string lightName = lightElem->Get<std::string>("name");
        auto lightEnt = _ecm.EntityByComponents(
            components::ParentEntity(_entity), components::Name(lightName));
        if (lightEnt != kNullEntity)
          updateLightElement(lightElem, _ecm, lightEnt);
        lightElem = lightElem->GetNextElement("light");
      }
    }

    return true;
  }

  /////////////////////////////////////////////////
  bool updateSensorElement(sdf::ElementPtr _elem,
                           const EntityComponentManager &_ecm,
                           const Entity &_entity)
  {
    // Update sdf based on current components.
    // This list is to be updated as other components become updateable during
    // simulation
    auto updateSensorNameAndPose = [&]
    {
      // override name and pose sdf element using values from ECM
      auto *nameComp = _ecm.Component<components::Name>(_entity);
      _elem->GetAttribute("name")->Set(nameComp->Data());

      auto *poseComp = _ecm.Component<components::Pose>(_entity);
      auto poseElem = _elem->GetElement("pose");

      // Remove all attributes of poseElem
      for (const auto *attrName : {"relative_to", "degrees", "rotation_format"})
      {
        sdf::ParamPtr attr = poseElem->GetAttribute(attrName);
        if (nullptr != attr)
        {
          attr->Reset();
        }
      }
      poseElem->Set(poseComp->Data());
      return true;
    };

    // camera
    auto camComp = _ecm.Component<components::Camera>(_entity);
    if (camComp)
    {
      const sdf::Sensor &sensor = camComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // depth camera
    auto depthCamComp = _ecm.Component<components::DepthCamera>(_entity);
    if (depthCamComp)
    {
      const sdf::Sensor &sensor = depthCamComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // thermal camera
    auto thermalCamComp = _ecm.Component<components::ThermalCamera>(_entity);
    if (thermalCamComp)
    {
      const sdf::Sensor &sensor = thermalCamComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // logical camera
    auto logicalCamComp = _ecm.Component<components::LogicalCamera>(_entity);
    if (logicalCamComp)
    {
      // components::LogicalCamera holds an sdf::ElementPtr instead of an
      // sdf::Sensor
      _elem = logicalCamComp->Data();
      return updateSensorNameAndPose();
    }
    // segmentation camera
    auto segmentationCamComp =
        _ecm.Component<components::SegmentationCamera>(_entity);
    if (segmentationCamComp)
    {
      const sdf::Sensor &sensor = segmentationCamComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }

    // gpu lidar
    auto gpuLidarComp = _ecm.Component<components::GpuLidar>(_entity);
    if (gpuLidarComp)
    {
      const sdf::Sensor &sensor = gpuLidarComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // altimeter
    auto altimeterComp = _ecm.Component<components::Altimeter>(_entity);
    if (altimeterComp)
    {
      const sdf::Sensor &sensor = altimeterComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // contact
    auto contactComp = _ecm.Component<components::ContactSensor>(_entity);
    if (contactComp)
    {
      // components::ContactSensor holds an sdf::ElementPtr instead of an
      // sdf::Sensor
      _elem = contactComp->Data();
      return updateSensorNameAndPose();
    }
    // air pressure
    auto airPressureComp =
        _ecm.Component<components::AirPressureSensor>(_entity);
    if (airPressureComp)
    {
      const sdf::Sensor &sensor = airPressureComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // air speed
    auto airSpeedComp =
        _ecm.Component<components::AirSpeedSensor>(_entity);
    if (airSpeedComp)
    {
      const sdf::Sensor &sensor = airSpeedComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // force torque
    auto forceTorqueComp = _ecm.Component<components::ForceTorque>(_entity);
    if (forceTorqueComp)
    {
      const sdf::Sensor &sensor = forceTorqueComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // imu
    auto imuComp = _ecm.Component<components::Imu>(_entity);
    if (imuComp)
    {
      const sdf::Sensor &sensor = imuComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }
    // magnetometer
    auto magnetometerComp =
        _ecm.Component<components::Magnetometer>(_entity);
    if (magnetometerComp)
    {
      const sdf::Sensor &sensor = magnetometerComp->Data();
      _elem->Copy(sensor.ToElement());
      return updateSensorNameAndPose();
    }

    return true;
  }

  /////////////////////////////////////////////////
  bool updateLightElement(sdf::ElementPtr _elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity)
  {
    // Update sdf based on the light component
    auto updateLightNameAndPose = [&]
    {
      // override name and pose sdf element using values from ECM
      auto *nameComp = _ecm.Component<components::Name>(_entity);
      _elem->GetAttribute("name")->Set(nameComp->Data());

      auto *poseComp = _ecm.Component<components::Pose>(_entity);
      auto poseElem = _elem->GetElement("pose");

      // Remove all attributes of poseElem
      for (const auto *attrName : {"relative_to", "degrees", "rotation_format"})
      {
        sdf::ParamPtr attr = poseElem->GetAttribute(attrName);
        if (nullptr != attr)
        {
          attr->Reset();
        }
      }
      poseElem->Set(poseComp->Data());
      return true;
    };

    // light
    auto lightComp = _ecm.Component<components::Light>(_entity);
    if (lightComp)
    {
      const sdf::Light &light = lightComp->Data();
      _elem->Copy(light.ToElement());
      return updateLightNameAndPose();
    }
    return true;
  }

  /////////////////////////////////////////////////
  bool updateJointElement(sdf::ElementPtr _elem,
                          const EntityComponentManager &_ecm,
                          const Entity &_entity)
  {
    // Update sdf based on the joint component
    auto updateJointNameAndPose = [&]
    {
      // override name and pose sdf element using values from ECM
      auto *nameComp = _ecm.Component<components::Name>(_entity);
      _elem->GetAttribute("name")->Set(nameComp->Data());

      auto *poseComp = _ecm.Component<components::Pose>(_entity);
      auto poseElem = _elem->GetElement("pose");

      // Remove all attributes of poseElem
      for (const auto *attrName : {"relative_to", "degrees", "rotation_format"})
      {
        sdf::ParamPtr attr = poseElem->GetAttribute(attrName);
        if (nullptr != attr)
        {
          attr->Reset();
        }
      }
      poseElem->Set(poseComp->Data());
      return true;
    };

    // joint
    auto jointComp = _ecm.Component<components::Joint>(_entity);
    if (!jointComp)
    {
      return false;
    }

    // joint type
    auto jointTypeComp = _ecm.Component<components::JointType>(_entity);
    sdf::JointType jointType = jointTypeComp->Data();
    if (jointTypeComp)
    {
      std::string jointTypeStr = "invalid";
      switch (jointType)
      {
        case sdf::JointType::BALL:
          jointTypeStr = "ball";
          break;
        case sdf::JointType::CONTINUOUS:
          jointTypeStr = "continuous";
          break;
        case sdf::JointType::FIXED:
          jointTypeStr = "fixed";
          break;
        case sdf::JointType::PRISMATIC:
          jointTypeStr = "prismatic";
          break;
        case sdf::JointType::GEARBOX:
          jointTypeStr = "gearbox";
          break;
        case sdf::JointType::REVOLUTE:
          jointTypeStr = "revolute";
          break;
        case sdf::JointType::REVOLUTE2:
          jointTypeStr = "revolute2";
          break;
        case sdf::JointType::SCREW:
          jointTypeStr = "screw";
          break;
        case sdf::JointType::UNIVERSAL:
          jointTypeStr = "universal";
          break;
        default:
          break;
      }
      _elem->GetAttribute("type")->Set<std::string>(jointTypeStr);
    }

    // parent
    auto parentLinkNameComp =
        _ecm.Component<components::ParentLinkName>(_entity);
    if (parentLinkNameComp)
    {
      _elem->GetElement("parent")->Set<std::string>(parentLinkNameComp->Data());
    }
    // child
    auto childLinkNameComp = _ecm.Component<components::ChildLinkName>(_entity);
    if (childLinkNameComp)
    {
      _elem->GetElement("child")->Set<std::string>(childLinkNameComp->Data());
    }
    // thread pitch
    auto threadPitchComp = _ecm.Component<components::ThreadPitch>(_entity);
    if (threadPitchComp && jointType == sdf::JointType::SCREW)
    {
      _elem->GetElement("thread_pitch")->Set<double>(threadPitchComp->Data());
    }
    // axis
    auto jointAxisComp = _ecm.Component<components::JointAxis>(_entity);
    if (jointAxisComp)
    {
      const sdf::JointAxis axis = jointAxisComp->Data();
      _elem->GetElement("axis")->Copy(axis.ToElement());
    }
    // axis2
    auto jointAxis2Comp = _ecm.Component<components::JointAxis2>(_entity);
    if (jointAxis2Comp)
    {
      const sdf::JointAxis axis2 = jointAxis2Comp->Data();
      _elem->GetElement("axis2")->Copy(axis2.ToElement(1u));
    }

    // sensors
    // remove existing ones in sdf element and add new ones from ECM.
    std::vector<sdf::ElementPtr> toRemove;
    if (_elem->HasElement("sensor"))
    {
      for (auto sensorElem = _elem->GetElement("sensor"); sensorElem;
           sensorElem = sensorElem->GetNextElement("sensor"))
      {
        toRemove.push_back(sensorElem);
      }
    }
    for (const auto &e : toRemove)
    {
      _elem->RemoveChild(e);
    }

    auto sensorEntities = _ecm.EntitiesByComponents(
        components::ParentEntity(_entity), components::Sensor());

    for (const auto &sensorEnt : sensorEntities)
    {
      sdf::ElementPtr sensorElem = _elem->AddElement("sensor");
      updateSensorElement(sensorElem, _ecm, sensorEnt);
    }

    return updateJointNameAndPose();
  }

  /////////////////////////////////////////////////
  /// \brief Checks if a string is a number
  /// \param[in] _str The string to check
  /// \return True if the string is a number
  bool isNumber(const std::string &_str)
  {
    for (const char &c : _str)
      if (!std::isdigit(c)) return false;

    return true;
  }

  /////////////////////////////////////////////////
  void updateModelElementWithNestedInclude(sdf::ElementPtr &_elem,
                                           const bool _saveFuelVersion,
                                           const IncludeUriMap &_includeUriMap)
  {
    sdf::ElementPtr e = _elem->GetFirstElement(), nextE = nullptr;
    while (e != nullptr)
    {
      nextE = e->GetNextElement();

      if (e->GetIncludeElement() != nullptr)
      {
        std::string modelDir = common::parentPath(e->FilePath());
        auto uriMapIt = _includeUriMap.find(modelDir);

        if (_saveFuelVersion && uriMapIt != _includeUriMap.end())
        {
          // find fuel model version from file path
          std::string version = common::basename(modelDir);

          if (isNumber(version))
          {
            std::string uri = e->GetIncludeElement()->Get<std::string>("uri");
            uri = uri + "/" + version;
            e->GetIncludeElement()->GetElement("uri")->Set(uri);
          }
          else
          {
            gzwarn << "Error retrieving Fuel model version,"
                    << " saving model without version."
                    << std::endl;
          }
        }

        e->RemoveAllAttributes();
        e->Copy(e->GetIncludeElement());
      }
      else if (e->GetName() == "model")
      {
        updateModelElementWithNestedInclude(e,
              _saveFuelVersion, _includeUriMap);
      }

      e = nextE;
    }
  }

  /////////////////////////////////////////////////
  bool updateIncludeElement(const sdf::ElementPtr &_elem,
                            const EntityComponentManager &_ecm,
                            const Entity &_entity, const std::string &_uri)
  {
    _elem->GetElement("uri")->Set(_uri);

    auto *nameComp = _ecm.Component<components::Name>(_entity);
    _elem->GetElement("name")->Set(nameComp->Data());

    auto *poseComp = _ecm.Component<components::Pose>(_entity);

    auto poseElem = _elem->GetElement("pose");

    // Remove all attributes of poseElem
    for (const auto *attrName : {"relative_to", "degrees", "rotation_format"})
    {
      sdf::ParamPtr attr = poseElem->GetAttribute(attrName);
      if (nullptr != attr)
      {
        attr->Reset();
      }
    }
    poseElem->Set(poseComp->Data());
    return true;
  }
}
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
