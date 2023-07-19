/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
#ifndef GZ_SIM_LIGHT_HH_
#define GZ_SIM_LIGHT_HH_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/utils/ImplPtr.hh>

#include <gz/math/Angle.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/sim/config.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Export.hh"
#include "gz/sim/Types.hh"

namespace gz
{
  namespace sim
  {
    // Inline bracket to help doxygen filtering.
    inline namespace GZ_SIM_VERSION_NAMESPACE {
    //
    /// \class Light Light.hh gz/sim/Light.hh
    /// \brief This class provides wrappers around entities and components
    /// which are more convenient and straight-forward to use than dealing
    /// with the `EntityComponentManager` directly.
    /// All the functions provided here are meant to be used with a light
    /// entity.
    ///
    /// For example, given a light's entity, find the value of its
    /// name component, one could use the entity-component manager (`ecm`)
    /// directly as follows:
    ///
    ///     std::string name = ecm.Component<components::Name>(entity)->Data();
    ///
    /// Using this class however, the same information can be obtained with
    /// a simpler function call:
    ///
    ///    Light light(entity);
    ///    std::string name = light.Name(ecm);
    ///
    class GZ_SIM_VISIBLE Light
    {
      /// \brief Constructor
      /// \param[in] _entity Light entity
      public: explicit Light(sim::Entity _entity = kNullEntity);

      /// \brief Get the entity which this Light is related to.
      /// \return Light entity.
      public: sim::Entity Entity() const;

      /// \brief Reset Entity to a new one
      /// \param[in] _newEntity New light entity.
      public: void ResetEntity(sim::Entity _newEntity);

      /// \brief Check whether this light correctly refers to an entity that
      /// has a components::Light.
      /// \param[in] _ecm Entity-component manager.
      /// \return True if it's a valid light in the manager.
      public: bool Valid(const EntityComponentManager &_ecm) const;

      /// \brief Get the light's unscoped name.
      /// \param[in] _ecm Entity-component manager.
      /// \return Light's name or nullopt if the entity does not have a
      /// components::Name component
      public: std::optional<std::string> Name(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the pose of the light.
      /// The pose is given w.r.t the light's parent. which can be a world or
      /// a link.
      /// \param[in] _ecm Entity-component manager.
      /// \return Pose of the light or nullopt if the entity does not
      /// have a components::Pose component.
      public: std::optional<math::Pose3d> Pose(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light type
      /// \param[in] _ecm Entity-component manager.
      /// \return Type of the light or nullopt if the entity does not
      /// have a components::LightType component.
      public: std::optional<std::string> Type(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light diffuse color
      /// \param[in] _ecm Entity-component manager.
      /// \return Diffuse color of the light or nullopt if the entity does not
      /// have a components::Light component.
      public: std::optional<math::Color> DiffuseColor(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light specular color
      /// \param[in] _ecm Entity-component manager.
      /// \return Specular color of the light or nullopt if the entity does not
      /// have a components::Light component.
      public: std::optional<math::Color> SpecularColor(
          const EntityComponentManager &_ecm) const;

      /// \brief Get whether the light casts shadows
      /// \param[in] _ecm Entity-component manager.
      /// \return Cast shadow bool value of light or nullopt if the entity does
      /// not have a components::CastShadows component.
      public: std::optional<bool> CastShadows(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light intensity.
      /// \param[in] _ecm Entity-component manager.
      /// \return Intensity of light or nullopt if the entity does not
      /// have a components::Light component.
      public: std::optional<double> Intensity(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light direction.
      /// \param[in] _ecm Entity-component manager.
      /// \return Direction of light or nullopt if the entity does not
      /// have a components::Light component.
      public: std::optional<math::Vector3d> Direction(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light attenuation range.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _ecm Entity-component manager.
      /// \return Attenuation range of light or nullopt if the entity does not
      /// have a components::Light component.
      public: std::optional<double> AttenuationRange(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light attenuation constant value.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _ecm Entity-component manager.
      /// \return Attenuation constant value of light or nullopt if the entity
      /// does not have a components::Light component.
      public: std::optional<double> AttenuationConstant(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light attenuation linear value.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _ecm Entity-component manager.
      /// \return Attenuation linear value of light or nullopt if the entity
      /// does not have a components::Light component.
      public: std::optional<double> AttenuationLinear(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the light attenuation quadratic value.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _ecm Entity-component manager.
      /// \return Attenuation quadratic value of light or nullopt if the entity
      /// does not have a components::Light component.
      public: std::optional<double> AttenuationQuadratic(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the inner angle of light. Applies to spot lights only.
      /// \param[in] _ecm Entity-component manager.
      /// \return Inner angle of spot light or nullopt if the entity
      /// does not have a components::Light component.
      public: std::optional<math::Angle> SpotInnerAngle(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the outer angle of light. Applies to spot lights only.
      /// \param[in] _ecm Entity-component manager.
      /// \return Outer angle of spot light or nullopt if the entity
      /// does not have a components::Light component.
      public: std::optional<math::Angle> SpotOuterAngle(
          const EntityComponentManager &_ecm) const;

      /// \brief Get the fall off value of light. Applies to spot lights only.
      /// \param[in] _ecm Entity-component manager.
      /// \return Fall off value of spot light or nullopt if the entity
      /// does not have a components::Light component.
      public: std::optional<double> SpotFalloff(
          const EntityComponentManager &_ecm) const;

      /// \brief Set the pose of this light.
      /// \param[in] _ecm Entity-component manager.
      /// Pose is set w.r.t. the light's parent which can be a world or a link.
      /// \param[in] _pose Pose to set the light to.
      public: void SetPose(EntityComponentManager &_ecm,
          const math::Pose3d &_pose);

      /// \brief Set the diffuse color of this light.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _color Diffuse color to set the light to
      public: void SetDiffuseColor(EntityComponentManager &_ecm,
          const math::Color &_color);

      /// \brief Set the specular color of this light.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _color Specular color to set the light to
      public: void SetSpecularColor(EntityComponentManager &_ecm,
          const math::Color &_color);

      /// \brief Set whether the light casts shadows.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _bool True to cast shadows, false to not cast shadows.
      public: void SetCastShadows(EntityComponentManager &_ecm,
          bool _castShadows);

      /// \brief Set light intensity.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _value Intensity value
      public: void SetIntensity(EntityComponentManager &_ecm,
          double _value);

      /// \brief Set light direction. Applies to directional lights
      /// \param[in] _ecm Entity-component manager.
      /// and spot lights only.
      /// \param[in] _dir Direction to set
      public: void SetDirection(EntityComponentManager &_ecm,
          const math::Vector3d &_dir);

      /// \brief Set attenuation range of this light.
      /// \param[in] _ecm Entity-component manager.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _range Attenuation range value to set.
      public: void SetAttenuationRange(EntityComponentManager &_ecm,
          double _range);

      /// \brief Set attenuation constant value of this light.
      /// \param[in] _ecm Entity-component manager.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _value Attenuation constant value to set
      public: void SetAttenuationConstant(EntityComponentManager &_ecm,
          double _value);

      /// \brief Set attenuation linear value of this light.
      /// \param[in] _ecm Entity-component manager.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _value Attenuation linear value to set
      public: void SetAttenuationLinear(EntityComponentManager &_ecm,
          double _value);

      /// \brief Set attenuation quadratic value of this light.
      /// \param[in] _ecm Entity-component manager.
      /// Light attenuation is not applicable to directional lights.
      /// \param[in] _value Attenuation quadratic value to set
      public: void SetAttenuationQuadratic(EntityComponentManager &_ecm,
          double _value);

      /// \brief Set inner angle for this light. Applies to spot lights only.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _angle Angle to set.
      public: void SetSpotInnerAngle(EntityComponentManager &_ecm,
          const math::Angle &_angle);

      /// \brief Set outer angle for this light. Applies to spot lights only.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _angle Angle to set.
      public: void SetSpotOuterAngle(EntityComponentManager &_ecm,
          const math::Angle &_angle);

      /// \brief Set fall off value for this light. Applies to spot lights only.
      /// \param[in] _ecm Entity-component manager.
      /// \param[in] _fallOff Fall off value
      public: void SetSpotFalloff(EntityComponentManager &_ecm,
          double _falloff);

      /// \brief Get the parent entity. This can be a world or a link.
      /// \param[in] _ecm Entity-component manager.
      /// \return Parent entity or nullopt if the entity does not have a
      /// components::ParentEntity component.
      public: std::optional<sim::Entity> Parent(
          const EntityComponentManager &_ecm) const;

      /// \brief Private data pointer.
      GZ_UTILS_IMPL_PTR(dataPtr)
    };
    }
  }
}
#endif
