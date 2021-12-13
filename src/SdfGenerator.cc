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

#include <ignition/common/URI.hh>

#include "ignition/gazebo/Util.hh"
#include "ignition/gazebo/components/AirPressureSensor.hh"
#include "ignition/gazebo/components/Altimeter.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/ContactSensor.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/ForceTorque.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/Imu.hh"
#include "ignition/gazebo/components/Inertial.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/LogicalCamera.hh"
#include "ignition/gazebo/components/Magnetometer.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/components/SelfCollide.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/SegmentationCamera.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/ThermalCamera.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/components/World.hh"


namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE
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
    // TODO(addisu) Remove actors and lights
    std::vector<sdf::ElementPtr> toRemove;
    if (_elem->HasElement("model"))
    {
      for (auto modelElem = _elem->GetElement("model"); modelElem;
           modelElem = modelElem->GetNextElement("model"))
      {
        toRemove.push_back(modelElem);
      }
    }
    for (const auto &e : toRemove)
    {
      _elem->RemoveChild(e);
    }

    auto worldDir = common::parentPath(worldSdf->Data().Element()->FilePath());

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
              // $HOME/.ignition/fuel/example.org/test/models/Backpack/2/
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
            ignwarn << "Error retrieving Fuel model version,"
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

  /////////////////////////////////////////////////
  std::optional<sdf::World> generateWorldSdf(
      const EntityComponentManager &_ecm, const Entity &_entity,
      const IncludeUriMap &_includeUriMap,
      const msgs::SdfGeneratorConfig &_config)
  {
    const auto *worldSdf = _ecm.Component<components::WorldSdf>(_entity);

    if (nullptr == worldSdf)
      return std::nullopt;

    sdf::World world(worldSdf->Data());

    // First remove child entities of <world> whose names can be changed during
    // simulation (eg. models). Then we add them back from the data in the
    // ECM.
    world.ClearModels();
    world.ClearActors();
    world.ClearLights();

    auto worldDir = common::parentPath(worldSdf->Data().Element()->FilePath());

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
            std::optional<sdf::Model> model = generateModelSdf(
                _ecm, _modelEntity);
            if (model)
              world.AddModel(*model);

            // Check & update possible //model/include(s)
            /*if (!modelConfig.expand_include_tags().data())
            {
              updateModelElementWithNestedInclude(modelElem,
                    modelConfig.save_fuel_version().data(), _includeUriMap);
            }*/
          }
          else if (uriMapIt != _includeUriMap.end())
          {/*
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
              // $HOME/.ignition/fuel/example.org/test/models/Backpack/2/
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
            */
          }
          else
          {/*
            // The model is not in the includeUriMap, but expandIncludeTags =
            // false, so we will assume that its uri is the file path of the
            // model on the local machine
            auto includeElem = _elem->AddElement("include");
            const std::string uri = "file://" + modelDir;
            updateIncludeElement(includeElem, _ecm, _modelEntity, uri);
            */
          }
          return true;
        });

    return world;
  }

  /////////////////////////////////////////////////
  std::optional<sdf::Model> generateModelSdf(const EntityComponentManager &_ecm,
      const Entity &_entity)
  {
    const auto *modelSdf = _ecm.Component<components::ModelSdf>(_entity);

    if (nullptr == modelSdf)
      return std::nullopt;

    // Copy the model, then update its values.
    sdf::Model model(modelSdf->Data());

    // Update sdf based on current components. Here are the list of components
    // to be updated:
    // - Name
    // - Pose
    // - Static
    // - SelfCollide
    // This list is to be updated as other components become updateable during
    // simulation
    auto *nameComp = _ecm.Component<components::Name>(_entity);
    if (nameComp)
      model.SetName(nameComp->Data());

    auto *poseComp = _ecm.Component<components::Pose>(_entity);
    if (poseComp)
      model.SetRawPose(poseComp->Data());

    // static
    auto *staticComp = _ecm.Component<components::Static>(_entity);
    if (staticComp)
      model.SetStatic(staticComp->Data());

    // self collide
    auto *selfCollideComp = _ecm.Component<components::SelfCollide>(_entity);
    if (selfCollideComp)
     model.SetSelfCollide(selfCollideComp->Data());

    // Update links
    std::vector<Entity> modelLinks = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Link());

    model.ClearLinks();
    for (Entity &link : modelLinks)
    {
      std::optional<sdf::Link> linkSdf = generateLinkSdf(_ecm, link);
      if (linkSdf)
        model.AddLink(*linkSdf);
    }

    // Update joints
    std::vector<Entity> modelJoints = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Joint());

    model.ClearJoints();
    /*for (Entity &joint : modelJoints)
    {
      std::optional<sdf::Joint> jointSdf = generateJointSdf(_ecm, joint);
      if (jointSdf)
        model.AddJoint(*JoinSdf);
    }*/

    // Update nested models
    std::vector<Entity> nestedModels = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Model());

    model.ClearModels();
    for (Entity &nestedModel : nestedModels)
    {
      std::optional<sdf::Model> nestedModelSdf =
        generateModelSdf(_ecm, nestedModel);
      if (nestedModelSdf)
        model.AddModel(*nestedModelSdf);
    }

    return model;
  }

  /////////////////////////////////////////////////
  std::optional<sdf::Link> generateLinkSdf(const EntityComponentManager &_ecm,
      const Entity &_entity)
  {
    sdf::Link link;

    // Update sdf based on current components. Here are the list of components
    // to be updated:
    // - Name
    // - Pose
    // - Inertial
    // - WindMode
    // This list is to be updated as other components become updateable during
    // simulation
    auto *nameComp = _ecm.Component<components::Name>(_entity);
    if (nameComp)
      link.SetName(nameComp->Data());

    auto *poseComp = _ecm.Component<components::Pose>(_entity);
    if (poseComp)
      link.SetRawPose(poseComp->Data());

    // inertial
    auto inertialComp = _ecm.Component<components::Inertial>(_entity);
    if (inertialComp)
      link.SetInertial(inertialComp->Data());

    // wind mode
    auto windModeComp = _ecm.Component<components::WindMode>(_entity);
    if (windModeComp)
      link.SetEnableWind(windModeComp->Data());

    // Update sensors
    std::vector<Entity> sensors = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Sensor());

    link.ClearSensors();
    for (Entity &sensor : sensors)
    {
      std::optional<sdf::Sensor> sensorSdf = generateSensorSdf(_ecm, sensor);
      if (sensorSdf)
        link.AddSensor(*sensorSdf);
    }

    // Update collisions
    std::vector<Entity> collisions = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Collision());

    link.ClearCollisions();
    for (Entity &collision : collisions)
    {
      auto comp = _ecm.Component<components::CollisionElement>(collision);
      link.AddCollision(comp->Data());
    }

    // Update visuals
    std::vector<Entity> visuals = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Visual());

    link.ClearVisuals();
    for (Entity &visual : visuals)
    {
      auto comp = _ecm.Component<components::VisualElement>(visual);
      link.AddVisual(comp->Data());
    }

    // Update lights
    std::vector<Entity> lights = _ecm.EntitiesByComponents(
      components::ParentEntity(_entity), components::Light());

    link.ClearLights();
    for (Entity &light : lights)
    {
      auto comp = _ecm.Component<components::Light>(light);
      link.AddLight(comp->Data());
    }

    return link;
  }

  /////////////////////////////////////////////////
  std::optional<sdf::Sensor> generateSensorSdf(
      const EntityComponentManager &_ecm, const Entity &_entity)
  {
    // Update sdf based on current components.
    // This list is to be updated as other components become updateable during
    // simulation
    sdf::Sensor sensor;

    // camera
    auto camComp = _ecm.Component<components::Camera>(_entity);
    if (camComp)
      sensor = camComp->Data();
    // depth camera
    auto depthCamComp = _ecm.Component<components::DepthCamera>(_entity);
    if (depthCamComp)
      sensor = depthCamComp->Data();
    // thermal camera
    auto thermalCamComp = _ecm.Component<components::ThermalCamera>(_entity);
    if (thermalCamComp)
      sensor = thermalCamComp->Data();
    // logical camera
    // IMPLEMENT ME: auto logicalCamComp = _ecm.Component<components::LogicalCamera>(_entity);
    // if (logicalCamComp)
    //   sensor = logicalCamComp->Data();
    // segmentation camera
    auto segmentationCamComp =
        _ecm.Component<components::SegmentationCamera>(_entity);
    if (segmentationCamComp)
      sensor = segmentationCamComp->Data();
    // gpu lidar
    auto gpuLidarComp = _ecm.Component<components::GpuLidar>(_entity);
    if (gpuLidarComp)
      sensor = gpuLidarComp->Data();
    // altimeter
    auto altimeterComp = _ecm.Component<components::Altimeter>(_entity);
    if (altimeterComp)
      sensor = altimeterComp->Data();
    // contact
    auto contactComp = _ecm.Component<components::ContactSensor>(_entity);
    if (contactComp)
      sensor = altimeterComp->Data();
    // air pressure
    auto airPressureComp =
        _ecm.Component<components::AirPressureSensor>(_entity);
    if (airPressureComp)
      sensor = airPressureComp->Data();
    // force torque
    auto forceTorqueComp = _ecm.Component<components::ForceTorque>(_entity);
    if (forceTorqueComp)
      sensor = forceTorqueComp->Data();
    // imu
    auto imuComp = _ecm.Component<components::Imu>(_entity);
    if (imuComp)
      sensor = imuComp->Data();
    // magnetometer
    auto magnetometerComp =
        _ecm.Component<components::Magnetometer>(_entity);
    if (magnetometerComp)
      sensor = magnetometerComp->Data();

    // Update the name
    auto *nameComp = _ecm.Component<components::Name>(_entity);
    if (nameComp)
      sensor.SetName(nameComp->Data());

    // Update the pose
    auto *poseComp = _ecm.Component<components::Pose>(_entity);
    if (poseComp)
      sensor.SetRawPose(poseComp->Data());

    return sensor;
  }
}
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
