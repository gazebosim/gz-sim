/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "LookupWheelSlip.hh"

#include <cmath>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/common/Image.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Matrix4.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/parameters/Client.hh>

#include <sdf/Geometry.hh>
#include <sdf/Heightmap.hh>

#include "gz/sim/components/Geometry.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LookupWheelSlipPrivate
{
  /// \brief Can the nominal (original) surface param values from the parameter
  /// registry.
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: bool GetNominalSurfaceParams(const EntityComponentManager &_ecm);

  /// \brief Get the scoped param name for an entity
  /// \param[in] _entity Entity id.
  /// \param[in] _paramName Name of the parameter.
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  /// \return Scoped name of the parameter.
  public: std::string ScopedParamName(const Entity &_entity,
      const std::string_view &_paramName,
      const EntityComponentManager &_ecm) const;

  /// \brief Update the params in the parameter registry
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void UpdateParams(const EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief /Filename of the slip map texture
  public: std::string slipMapFilename;

  /// \brief The loaded slip map image
  public: common::Image slipMapImg;

  /// \brief RGB data of the slip map img;
  public: std::vector<unsigned char> slipMapRgb;

  /// \brief X dimension of the collision geometry in meters.
  public: double sizeX = 0.0;

  /// \brief Y dimension of the collision geometry in meters.
  public: double sizeY = 0.0;

  /// \brief The lateral slip delta to apply.
  public: double lateralSlipDelta = 0.05;

  /// \brief The lateral slip delta to apply.
  public: double longitudinalSlipDelta = 0.005;

  /// \brief The friction delta to apply.
  public: double frictionDelta = 0.5;

  /// \brief A transform from world to img space.
  public: math::Matrix4d worldToImgTransform;

  /// \brief True if the system is initialized, false otherwise
  public: bool initialized = false;

  /// \brief A unique set of link entities
  public: std::unordered_set<Entity> linkEntities;

  /// \brief The nominal surface param values.
  /// These are the original, non-modified param values.
  /// The elements are <param_name, param_value>
  public: std::unordered_map<std::string, double> nominalParamValues;

  /// \brief The new surface param values computed by this plugin. The params
  /// on the parameter registry will be updated by these values.
  /// The elements are <param_name, param_value_to_set>
  public: std::unordered_map<std::string, double> newParamValues;

  /// \brief Transport parameter client
  public: transport::parameters::ParametersClient client;

  /// \brief Lateral slip compliance parameter name in the parameter registry.
  /// The name needs to match the one in WheelSlip system.
  public: static constexpr std::string_view kSlipComplianceLateralParamName =
      "slip_compliance_lateral";

  /// \brief Longitudinal slip compliance parameter name in the parameter
  /// registry.
  /// The name needs to match the one in WheelSlip system.
  public: static constexpr std::string_view
      kSlipComplianceLongitudinalParamName = "slip_compliance_longitudinal";

  /// \brief Primary friction coefficient parameter name in the parameter
  /// registry.
  /// The name needs to match the one in WheelSlip system.
  public: static constexpr std::string_view
      kFrictionCoefficientPrimaryParamName = "friction_coefficient_primary";

  /// \brief Secondary friction coefficient parameter name in the parameter
  /// registry.
  /// The name needs to match the one in WheelSlip system.
  public: static constexpr std::string_view
      kFrictionCoefficientSecondaryParamName = "friction_coefficient_secondary";

  /// \brief Lateral slip color channel representation (red channel)
  public: static constexpr unsigned int kLateralColorChannel = 0;

  /// \brief Longitudinal slip color channel representation (green channel)
  public: static constexpr unsigned int kLongitudinalColorChannel = 1;

  /// \brief Friction color channel representation (blue channel)
  public: static constexpr unsigned int kFrictionColorChannel = 2;

  /// \brief Nominal color value - half of 8 bit
  public: static constexpr unsigned int kNominalColor = 1 << 7;
};

//////////////////////////////////////////////////
LookupWheelSlip::LookupWheelSlip()
  : dataPtr(std::make_unique<LookupWheelSlipPrivate>())
{
}

//////////////////////////////////////////////////
void LookupWheelSlip::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "LookupWheelSlip plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  auto slipMapElem = _sdf->FindElement("slip_map");
  if (!slipMapElem)
  {
    gzerr << "No 'slip_map' provided. Will not dynamically update "
          << "wheel slip values. " << std::endl;
    return;
  }
  this->dataPtr->slipMapFilename = slipMapElem->Get<std::string>();
  if (this->dataPtr->slipMapFilename.empty())
  {
    gzerr << "No value for 'slip_path' provided. Will not dynamically update "
          << "wheel slip values. " << std::endl;
    return;
  }

  // \todo(iche033) Auto determine size from region collision geometry?
  math::Vector3d colSize;
  /*
  for (const auto &linkEntity : regionModel.Links(_ecm))
  {
    sim::Link link(linkEntity);
    for (const auto &collisionEntity : link.Collisions(_ecm))
    {
      auto geomComp = _ecm.Component<components::Geometry>(collisionEntity);
      if (geomComp && geomComp->Data().Type() == sdf::GeometryType::HEIGHTMAP)
      {
        const sdf::Heightmap *shape = geomComp->Data().HeightmapShape();
        colSize = shape->Size();
        break;
      }
    }
  }
  */

  auto sizeXElem = _sdf->FindElement("size_x");
  if (sizeXElem)
  {
    this->dataPtr->sizeX = sizeXElem->Get<double>();
  }
  else if (colSize.X() > 0.0)
  {
    this->dataPtr->sizeX = colSize.X();
  }
  else
  {
    gzerr << "Unable to determine size x from collision and missing "
          << "'size_x' param. Will not dynamically update wheel slip values."
          << std::endl;
    return;
  }
  auto sizeYElem = _sdf->FindElement("size_y");
  if (sizeYElem)
  {
    this->dataPtr->sizeY = sizeYElem->Get<double>();
  }
  else if (colSize.Y() > 0.0)
  {
    this->dataPtr->sizeY = colSize.Y();
  }
  else
  {
    gzerr << "Unable to determine size y from collision and missing "
          << "'size_y' param. Will not dynamically update wheel slip values."
          << std::endl;
    return;
  }

  // \todo(iche033) Swap R and B channels?

  // transformation matrix from world to image coordinates
  std::string filePath;
  if (common::isFile(this->dataPtr->slipMapFilename))
  {
    filePath = this->dataPtr->slipMapFilename;
  }
  else if (common::isRelativePath(this->dataPtr->slipMapFilename))
  {
    auto *component =
        _ecm.Component<components::WorldSdf>(worldEntity(_ecm));
    const std::string rootPath =
        common::parentPath(component->Data().Element()->FilePath());
    std::string path = common::joinPaths(rootPath,
        this->dataPtr->slipMapFilename);
    if (common::isFile(path))
      filePath = path;
  }
  if (filePath.empty())
  {
    gzerr << "Unable to find slip_map file: " << this->dataPtr->slipMapFilename
          << std::endl;
    return;
  }
  gzdbg << "Using slip_map: " << filePath << std::endl;
  this->dataPtr->slipMapImg.Load(filePath);
  this->dataPtr->slipMapRgb = this->dataPtr->slipMapImg.RGBData();
  this->dataPtr->worldToImgTransform(0, 0) =
      this->dataPtr->sizeX /
      static_cast<double>(this->dataPtr->slipMapImg.Width());
  this->dataPtr->worldToImgTransform(0, 3) = -this->dataPtr->sizeX / 2.0;
  this->dataPtr->worldToImgTransform(1, 1) =
      (this->dataPtr->sizeY /
       static_cast<double>(this->dataPtr->slipMapImg.Height())) * cos(GZ_PI);
  this->dataPtr->worldToImgTransform(1, 2) = -sin(GZ_PI);
  this->dataPtr->worldToImgTransform(1, 3) = this->dataPtr->sizeY / 2.0;
  this->dataPtr->worldToImgTransform(2, 1) = sin(GZ_PI);
  this->dataPtr->worldToImgTransform(2, 2) = cos(GZ_PI);
  this->dataPtr->worldToImgTransform(3, 3) = 1.0;
  this->dataPtr->worldToImgTransform =
      this->dataPtr->worldToImgTransform.Inverse();

  std::unordered_set<std::string> wheelLinkNames;
  auto wheelLinkElem = _sdf->FindElement("wheel_link_name");
  while (wheelLinkElem)
  {
    wheelLinkNames.insert(wheelLinkElem->Get<std::string>());
    wheelLinkElem = wheelLinkElem->GetNextElement("wheel_link_name");
  }
  if (wheelLinkNames.empty())
  {
    gzerr << "Error loading 'wheel_slip_name' element. Unable to load plugin."
          << std::endl;
    return;
  }

  for (const auto &linkName : wheelLinkNames)
  {
    auto link = Link(this->dataPtr->model.LinkByName(_ecm, linkName));
    if (!link.Valid(_ecm))
    {
      gzerr << "Could not find link named [" << linkName
            << "] in model [" << this->dataPtr->model.Name(_ecm) << "]"
            << std::endl;
      continue;
    }
    this->dataPtr->linkEntities.insert(link.Entity());
  }

  auto latSlipDeltaElem = _sdf->FindElement("slip_compliance_lateral_delta");
  if (latSlipDeltaElem)
  {
    this->dataPtr->lateralSlipDelta = latSlipDeltaElem->Get<double>();
  }
  auto lonSlipDeltaElem = _sdf->FindElement(
      "slip_compliance_longitudinal_delta");
  if (lonSlipDeltaElem)
  {
    this->dataPtr->longitudinalSlipDelta = lonSlipDeltaElem->Get<double>();
  }
  auto frictionDeltaElem = _sdf->FindElement("friction_delta");
  if (frictionDeltaElem)
  {
    this->dataPtr->frictionDelta = frictionDeltaElem->Get<double>();
  }

  std::string ns = sim::scopedName(worldEntity(_ecm), _ecm, "/", true);
  this->dataPtr->client = transport::parameters::ParametersClient(ns);
  this->dataPtr->initialized = true;

  gzdbg << "LookupWheelSlip plugin params: \n";
  gzdbg << "slip_map: " << this->dataPtr->slipMapFilename<< "\n";
  gzdbg << "size_x: " << this->dataPtr->sizeX << "\n";
  gzdbg << "size_y: " << this->dataPtr->sizeY << "\n";
  gzdbg << "slip_compliance_lateral_delta: "
        << this->dataPtr->lateralSlipDelta<< "\n";
  gzdbg << "slip_compliance_longitudinal_delta: "
        << this->dataPtr->longitudinalSlipDelta<< "\n";
  gzdbg << "friction_delta: " << this->dataPtr->frictionDelta << "\n";
  gzdbg << "wheel link name(s): " << "\n";
  for (const auto &linkName : wheelLinkNames)
    gzdbg << "  " << linkName << "\n";
}

//////////////////////////////////////////////////
void LookupWheelSlip::PreUpdate(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
    return;

  if (this->dataPtr->nominalParamValues.empty() &&
      !this->dataPtr->GetNominalSurfaceParams(_ecm))
  {
    return;
  }

  this->dataPtr->UpdateParams(_ecm);
}

//////////////////////////////////////////////////
std::string LookupWheelSlipPrivate::ScopedParamName(
    const Entity &_entity,
    const std::string_view &_paramName,
    const EntityComponentManager &_ecm) const
{
  std::string prefix = std::string("WheelSlip.") +
                       sim::scopedName(_entity, _ecm, ".", false);
  return prefix + "." + std::string(_paramName);
}

//////////////////////////////////////////////////
bool LookupWheelSlipPrivate::GetNominalSurfaceParams(
    const EntityComponentManager &_ecm)
{
  bool receivedAllParams = true;
  for (const auto &linkEnt : this->linkEntities)
  {
    // Slip compliance lateral
    std::string paramName =
        this->ScopedParamName(linkEnt, kSlipComplianceLateralParamName, _ecm);
    msgs::Double msg;
    transport::parameters::ParameterResult result =
        this->client.Parameter(paramName, msg);
    if (!result)
    {
      receivedAllParams = false;
      gzerr << "Unable to get parameter " << paramName << std::endl;
    }
    else
    {
      this->nominalParamValues[paramName] = msg.data();
    }

    // Slip compliance longitudinal
    paramName =
        this->ScopedParamName(
        linkEnt, kSlipComplianceLongitudinalParamName, _ecm);
    result = this->client.Parameter(paramName, msg);
    if (!result)
    {
      receivedAllParams = false;
      gzerr << "Unable to get parameter " << paramName << std::endl;
    }
    else
    {
      this->nominalParamValues[paramName] = msg.data();
    }

    // Primary friction coeff
    paramName =
        this->ScopedParamName(
        linkEnt, kFrictionCoefficientPrimaryParamName, _ecm);
    result = this->client.Parameter(paramName, msg);
    if (!result)
    {
      receivedAllParams = false;
      gzerr << "Unable to get parameter " << paramName << std::endl;
    }
    else
    {
      this->nominalParamValues[paramName] = msg.data();
    }

    // Secondary friction coeff
    paramName =
        this->ScopedParamName(
        linkEnt, kFrictionCoefficientSecondaryParamName, _ecm);
    result = this->client.Parameter(paramName, msg);
    if (!result)
    {
      receivedAllParams = false;
      gzerr << "Unable to get parameter " << paramName << std::endl;
    }
    else
    {
      this->nominalParamValues[paramName] = msg.data();
    }
  }

  if (!receivedAllParams)
    this->nominalParamValues.clear();

  for (const auto &coeffs : this->nominalParamValues)
  {
    gzdbg  << "Received nominal param: " << coeffs.first
           << ": " << coeffs.second << std::endl;
  }
  return receivedAllParams;
}

//////////////////////////////////////////////////
void LookupWheelSlipPrivate::UpdateParams(
    const EntityComponentManager &_ecm)
{
  // std::cout << "UpdateParams" << std::endl;

  for (const auto &linkEnt : this->linkEntities)
  {
    sim::Link link(linkEnt);
    math::Vector3d linkWorldPos = link.WorldPose(_ecm).value().Pos();
    math::Vector3d imgPos = this->worldToImgTransform * linkWorldPos;

    int u = static_cast<int>(std::round(imgPos.X()));
    int v = static_cast<int>(std::round(imgPos.Y()));

    // check wheel isn't outside drivable bounds (ie. outside of slipMapImg)
    if (u < 0 || static_cast<unsigned int>(u) >= this->slipMapImg.Width() ||
        v < 0 || static_cast<unsigned int>(v) >= this->slipMapImg.Height())
      continue;

    std::string latParamName =
        this->ScopedParamName(linkEnt, kSlipComplianceLateralParamName, _ecm);
    std::string lonParamName =
        this->ScopedParamName(
        linkEnt, kSlipComplianceLongitudinalParamName, _ecm);
    std::string mu1ParamName =
        this->ScopedParamName(
        linkEnt, kFrictionCoefficientPrimaryParamName, _ecm);
    std::string mu2ParamName =
        this->ScopedParamName(
        linkEnt, kFrictionCoefficientSecondaryParamName, _ecm);

    const unsigned int channels = 3u;
    unsigned int idx = (v * this->slipMapImg.Height() + u) * channels;

    int latDeltaScale = static_cast<int>(
        this->slipMapRgb[idx + kLateralColorChannel] - kNominalColor);
    double latCoeff = this->nominalParamValues[latParamName] +
        (latDeltaScale * this->lateralSlipDelta);
    if (latCoeff < 0.0)
    {
      latCoeff = 0.0;
    }
    int lonDeltaScale = static_cast<int>(
        this->slipMapRgb[idx + kLongitudinalColorChannel] - kNominalColor);
    double lonCoeff = this->nominalParamValues[lonParamName] +
        (lonDeltaScale * this->longitudinalSlipDelta);
    if (lonCoeff < 0.0)
    {
      lonCoeff = 0.0;
    }
    int muDeltaScale = static_cast<int>(
        this->slipMapRgb[idx + kFrictionColorChannel] - kNominalColor);
    double mu1Coeff = this->nominalParamValues[mu1ParamName] +
        (muDeltaScale * this->frictionDelta);
    if (mu1Coeff < 0.0)
    {
      mu1Coeff = 0.0;
    }
    double mu2Coeff = this->nominalParamValues[mu2ParamName] +
        (muDeltaScale * this->frictionDelta);
    if (mu2Coeff < 0.0)
    {
      mu2Coeff = 0.0;
    }

    // gzmsg << "[u, v]: " << u << ", " << v << std::endl;
    // std::cout << "-------------------------------------" << std::endl;
    // std::cout << "color: "
    //       << static_cast<int>(slipMapRgb[idx + kLateralColorChannel])
    //       << ", "
    //       << static_cast<int>(slipMapRgb[idx + kLongitudinalColorChannel])
    //       << ", "
    //       << static_cast<int>(slipMapRgb[idx + kFrictionColorChannel])
    //       << std::endl;
    // gzmsg << "[lat|lon|mu|mu2] nominal: "
    //       << this->nominalParamValues[latParamName] << ", "
    //       << this->nominalParamValues[lonParamName] << ", "
    //       << this->nominalParamValues[mu1ParamName] << ", "
    //       << this->nominalParamValues[mu2ParamName] << std::endl;
    // gzmsg << "[lat|lon|friction] delta: "
    //       << latDeltaScale << ", "
    //       << lonDeltaScale << ",  "
    //       << muDeltaScale << std::endl;
    // gzmsg << "[lat|lon|mu|mu2] coeff: "
    //       << latCoeff << ", "
    //       << lonCoeff << ",  "
    //       << mu1Coeff << ",  "
    //       << mu2Coeff << std::endl;

    // Update surface params
    if (!math::equal(this->newParamValues[latParamName], latCoeff, 1e-6))
    {
      this->newParamValues[latParamName] = latCoeff;
      msgs::Double msg;
      msg.set_data(latCoeff);
      auto result = this->client.SetParameter(latParamName, msg);
      if (!result)
      {
        gzerr << "Error setting param " << latParamName << std::endl;
      }
    }
    if (!math::equal(this->newParamValues[lonParamName], lonCoeff, 1e-6))
    {
      this->newParamValues[lonParamName] = lonCoeff;
      msgs::Double msg;
      msg.set_data(lonCoeff);
      auto result = this->client.SetParameter(lonParamName, msg);
      if (!result)
      {
        gzerr << "Error setting param " << lonParamName << std::endl;
      }
    }
    if (!math::equal(this->newParamValues[mu1ParamName], mu1Coeff, 1e-6))
    {
      this->newParamValues[mu1ParamName] = mu1Coeff;
      msgs::Double msg;
      msg.set_data(mu1Coeff);
      auto result = this->client.SetParameter(mu1ParamName, msg);
      if (!result)
      {
        gzerr << "Error setting param " << mu1ParamName << std::endl;
      }
    }
    if (!math::equal(this->newParamValues[mu2ParamName], mu2Coeff, 1e-6))
    {
      this->newParamValues[mu2ParamName] = mu2Coeff;
      msgs::Double msg;
      msg.set_data(mu2Coeff);
      auto result = this->client.SetParameter(mu2ParamName, msg);
      if (!result)
      {
        gzerr << "Error setting param " << mu2ParamName << std::endl;
      }
    }
  }
}

GZ_ADD_PLUGIN(LookupWheelSlip,
              System,
              LookupWheelSlip::ISystemConfigure,
              LookupWheelSlip::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LookupWheelSlip,
                    "gz::sim::systems::LookupWheelSlip")
