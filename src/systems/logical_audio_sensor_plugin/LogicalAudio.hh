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
#ifndef GZ_SIM_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_LOGICALAUDIO_HH_
#define GZ_SIM_SYSTEMS_LOGICAL_AUDIO_SENSOR_PLUGIN_LOGICALAUDIO_HH_

#include <string>

#include <gz/sim/components/LogicalAudio.hh>
#include <gz/sim/config.hh>
#include <gz/sim/logicalaudiosensorplugin-system/Export.hh>
#include <gz/math/Pose3.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace logical_audio
{
  /// \brief Determines if an audio device can detect volume at a certain level.
  /// \param[in] _volumeLevel The volume level that the microphone is
  /// attempting to detect. This should be a value between 0.0 (no volume) and
  /// 1.0 (maximum volume).
  /// \param[in] _volumeDetectionThreshold The listening device's minimum audio
  /// detection volume. This should be a value between 0.0 and 1.0. A device
  /// with a low detection threshold can hear a wider range of audio than a
  /// device with a higher detection threshold.
  /// \return true if the listening device can detect volume at _volumeLevel,
  /// false otherwise.
  GZ_SIM_LOGICALAUDIOSENSORPLUGIN_SYSTEM_VISIBLE
  bool detect(double _volumeLevel, double _volumeDetectionThreshold);

  /// \brief Computes the volume level of an audio source at a certain location.
  /// \note Users should call
  /// logical_audio::ValidateInnerRadiusAndFalloffDistance and
  /// logical_audio::ValidateVolumeLevel before calling this method in order to
  /// prevent undefined behavior.
  /// \param[in] _playing Whether the audio source is playing or not.
  /// \param[in] _attenuationFunc The source's attenuation function.
  /// \param[in] _attenuationShape The source's attenuation shape.
  /// \param[in] _sourceEmissionVolume The source's emission volume level.
  /// \param[in] _innerRadius The source's inner radius, which should be >= 0.
  /// \param[in] _falloffDistance The source's falloffDistance, which should be
  /// greater than _innerRadius.
  /// \param[in] _sourcePose The source's pose.
  /// \param[in] _targetPose The pose where the volume level should be
  /// calculated.
  /// \return The volume level at this location.
  /// If the attenuation function or shape is undefined, -1.0 is returned.
  /// If the source is not playing, 0.0 is returned.
  GZ_SIM_LOGICALAUDIOSENSORPLUGIN_SYSTEM_VISIBLE
  double computeVolume(bool _playing,
      AttenuationFunction _attenuationFunc,
      AttenuationShape _attenuationShape,
      double _sourceEmissionVolume,
      double _innerRadius,
      double _falloffDistance,
      const gz::math::Pose3d &_sourcePose,
      const gz::math::Pose3d &_targetPose);

  /// \brief Set the attenuation function that matches the defined string.
  /// The string is not case sensitive, and must match the spelling
  /// of the values in AttenuationFunction. If the spelling does not match,
  /// the attenuation function is set as AttenuationFunction::Undefined.
  ///
  /// \em Example: to set the attenuation function to
  /// AttenuationFunction::Linear, the following are examples of valid
  /// strings: "linear", "Linear", "LINEAR"
  ///
  /// \param[out] _attenuationFunc A reference to the variable that stores
  /// the calculated attenuation function.
  /// \param[in] _str A string that should map to a value in
  /// AttenuationFunction.
  GZ_SIM_LOGICALAUDIOSENSORPLUGIN_SYSTEM_VISIBLE
  void setAttenuationFunction(AttenuationFunction &_attenuationFunc,
      std::string _str);

  /// \brief Set the attenuation shape that matches the defined string.
  /// The string is not case sensitive, and must match the spelling
  /// of the values in AttenuationShape. If the spelling does not match,
  /// the attenuation shape is set as AttenuationShape::Undefined.
  ///
  /// \em Example: to set the attenuation shape to AttenuationShape::Sphere,
  /// the following are examples of valid strings: "sphere", "Sphere", "SPHERE"
  ///
  /// \param[out] _attenuationShape A reference to the variable that stores the
  /// calculated attenuation shape.
  /// \param[in] _str A string that should map to a value in
  ///   AttenuationShape.
  GZ_SIM_LOGICALAUDIOSENSORPLUGIN_SYSTEM_VISIBLE
  void setAttenuationShape(AttenuationShape &_attenuationShape,
      std::string _str);

  /// \brief Validate the inner radius and falloff distance for an audio source.
  /// \param[in,out] _innerRadius The inner radius to set for the source.
  /// This value must be > 0.
  /// If the value of this parameter is < 0, the source's inner radius
  /// will be set to 0.
  /// \param[in,out] _falloffDistance The falloff distance to set for the
  /// source. This value must be greater than _innerRadius.
  /// If _falloffDistance < _innerRadius, _falloffDistance will be set to
  /// _innerRadius + 1 (assuming that _innerRadius is valid).
  GZ_SIM_LOGICALAUDIOSENSORPLUGIN_SYSTEM_VISIBLE
  void validateInnerRadiusAndFalloffDistance(double &_innerRadius,
      double &_falloffDistance);

  /// \brief Validate a source's emission volume level.
  /// \param[in,out] _volumeLevel The volume the source should play at.
  /// This parameter is checked (and possibly clipped) to ensure that it falls
  /// between 0.0 (0% volume) and 1.0 (100% volume).
  GZ_SIM_LOGICALAUDIOSENSORPLUGIN_SYSTEM_VISIBLE
  void validateVolumeLevel(double &_volumeLevel);
}
}
}
}

#endif
