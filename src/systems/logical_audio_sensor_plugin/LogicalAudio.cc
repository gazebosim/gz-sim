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

#include "LogicalAudio.hh"

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace logical_audio
{
  /// \brief A map to help convert user-input strings to the proper
  ///   attenuation function.
  const std::unordered_map<std::string, AttenuationFunction>
    kAttFuncMap {{"linear", AttenuationFunction::LINEAR}};

  /// \brief A map to help convert user-input strings to the proper
  ///   attenuation shape.
  const std::unordered_map<std::string, AttenuationShape>
    kAttShapeMap {{"sphere", AttenuationShape::SPHERE}};

  //////////////////////////////////////////////////
  bool detect(double _volumeLevel, double _volumeDetectionThreshold)
  {
    // if the volume level is <= 0, this can't be detected
    // (not even if _volumeDetectionThreshold is 0.0)
    if (_volumeLevel < 0.00001)
      return false;

    return _volumeLevel >= _volumeDetectionThreshold;
  }

  //////////////////////////////////////////////////
  double computeVolume(bool _playing,
      AttenuationFunction _attenuationFunc,
      AttenuationShape _attenuationShape,
      double _sourceEmissionVolume,
      double _innerRadius,
      double _falloffDistance,
      const gz::math::Pose3d &_sourcePose,
      const gz::math::Pose3d &_targetPose)
  {
    if (!_playing)
      return 0.0;

    // make sure the source has a valid attenuation function and shape
    if ((_attenuationFunc == AttenuationFunction::UNDEFINED) ||
        (_attenuationShape == AttenuationShape::UNDEFINED))
      return -1.0;
    // make sure the audio source has a playing volume that's > 0
    else if (_sourceEmissionVolume < 0.00001)
      return 0.0;

    auto dist = _sourcePose.Pos().Distance(_targetPose.Pos());

    // Implementing AttenuationShape::SPHERE for now since that's
    // the only attenuation shape that's available
    if (dist <= _innerRadius)
      return _sourceEmissionVolume;
    else if (dist >= _falloffDistance)
      return 0.0;

    // Implementing AttenuationFunction::LINEAR for now since that's
    // the only attenuation function that's available.
    //
    // The equation below was calculated as follows:
    // Point slope formula is y - y_1 = m * (x - x_1), rewritten as:
    // y = (m * (x - x_1)) + y_1
    // The variables in the equation above are defined as:
    //    y = volume at _targetPose, m = slope,
    //    x = distance(_sourcePose, _targetPose),
    //    x_1 = _innerRadius, y_1 = _sourceEmissionVolume
    // The slope (m) is defined as delta_y / delta_x, where y is volume and x
    //    is position. We have two (x,y) points: y is the source's output
    //    volume when x is _innerRadius, and y is 0 when x is _falloffDistance.
    //    So, we can calculate the slope using these two points.
    double m = -_sourceEmissionVolume / (_falloffDistance - _innerRadius);
    return (m * (dist - _innerRadius)) + _sourceEmissionVolume;
  }

  //////////////////////////////////////////////////
  void setAttenuationFunction(AttenuationFunction &_attenuationFunc,
      std::string _str)
  {
    std::transform(_str.begin(), _str.end(),
        _str.begin(), ::tolower);

    auto iter = kAttFuncMap.find(_str);
    if (iter != kAttFuncMap.end())
      _attenuationFunc = iter->second;
    else
      _attenuationFunc = AttenuationFunction::UNDEFINED;
  }

  //////////////////////////////////////////////////
  void setAttenuationShape(AttenuationShape &_attenuationShape,
      std::string _str)
  {
    std::transform(_str.begin(), _str.end(),
        _str.begin(), ::tolower);

    auto iter = kAttShapeMap.find(_str);
    if (iter != kAttShapeMap.end())
      _attenuationShape = iter->second;
    else
      _attenuationShape = AttenuationShape::UNDEFINED;
  }

  //////////////////////////////////////////////////
  void validateInnerRadiusAndFalloffDistance(double &_innerRadius,
      double &_falloffDistance)
  {
    if (_innerRadius < 0.0)
    {
      _innerRadius = 0.0;
    }

    if (_falloffDistance <= _innerRadius)
    {
      _falloffDistance = _innerRadius + 1.0;
    }
  }

  //////////////////////////////////////////////////
  void validateVolumeLevel(double &_volumeLevel)
  {
    if (_volumeLevel < 0.0)
      _volumeLevel = 0.0;
    else if (_volumeLevel > 1.0)
      _volumeLevel = 1.0;
  }
}  // namespace logical_audio
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
