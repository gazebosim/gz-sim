 /*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <numeric>
#include <limits>
#include <deque>
#include "gz/math/RollingMean.hh"

using namespace gz::math;

/// \brief Private data
class gz::math::RollingMean::Implementation
{
  /// \brief The window size
  public: size_t windowSize{10};

  /// \brief The values.
  public: std::deque<double> values;
};

//////////////////////////////////////////////////
RollingMean::RollingMean(size_t _windowSize)
  : dataPtr(gz::utils::MakeImpl<Implementation>())
{
  if (_windowSize > 0)
    this->dataPtr->windowSize = _windowSize;
}

//////////////////////////////////////////////////
double RollingMean::Mean() const
{
  if (!this->dataPtr->values.empty())
  {
    return std::accumulate(
        this->dataPtr->values.begin(), this->dataPtr->values.end(), 0.0) /
      this->dataPtr->values.size();
  }

  return std::numeric_limits<double>::quiet_NaN();
}

//////////////////////////////////////////////////
size_t RollingMean::Count() const
{
  return this->dataPtr->values.size();
}

//////////////////////////////////////////////////
void RollingMean::Push(double _value)
{
  this->dataPtr->values.push_back(_value);
  while (this->dataPtr->values.size() > this->dataPtr->windowSize &&
         !this->dataPtr->values.empty())
  {
    this->dataPtr->values.pop_front();
  }
}

//////////////////////////////////////////////////
void RollingMean::Clear()
{
  this->dataPtr->values.clear();
}

//////////////////////////////////////////////////
void RollingMean::SetWindowSize(size_t _windowSize)
{
  if (_windowSize > 0)
  {
    this->dataPtr->windowSize = _windowSize;
    this->Clear();
  }
}

//////////////////////////////////////////////////
size_t RollingMean::WindowSize() const
{
  return this->dataPtr->windowSize;
}
