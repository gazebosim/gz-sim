/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include "gazebo/math/Vector3StatsPrivate.hh"
#include "gazebo/math/Vector3Stats.hh"

using namespace gazebo;
using namespace math;

//////////////////////////////////////////////////
Vector3Stats::Vector3Stats()
  : dataPtr(new Vector3StatsPrivate)
{
}

//////////////////////////////////////////////////
Vector3Stats::~Vector3Stats()
{
}

//////////////////////////////////////////////////
void Vector3Stats::InsertData(const Vector3 &_data)
{
  this->dataPtr->x.InsertData(_data.x);
  this->dataPtr->y.InsertData(_data.y);
  this->dataPtr->z.InsertData(_data.z);
  this->dataPtr->mag.InsertData(_data.GetLength());
}

//////////////////////////////////////////////////
bool Vector3Stats::InsertStatistic(const std::string &_name)
{
  bool x = this->dataPtr->x.InsertStatistic(_name);
  bool y = this->dataPtr->y.InsertStatistic(_name);
  bool z = this->dataPtr->z.InsertStatistic(_name);
  bool mag = this->dataPtr->mag.InsertStatistic(_name);
  return x && y && z && mag;
}

//////////////////////////////////////////////////
bool Vector3Stats::InsertStatistics(const std::string &_names)
{
  bool x = this->dataPtr->x.InsertStatistics(_names);
  bool y = this->dataPtr->y.InsertStatistics(_names);
  bool z = this->dataPtr->z.InsertStatistics(_names);
  bool mag = this->dataPtr->mag.InsertStatistics(_names);
  return x && y && z && mag;
}

//////////////////////////////////////////////////
void Vector3Stats::Reset()
{
  this->dataPtr->x.Reset();
  this->dataPtr->y.Reset();
  this->dataPtr->z.Reset();
  this->dataPtr->mag.Reset();
}

//////////////////////////////////////////////////
SignalStats Vector3Stats::X() const
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
SignalStats Vector3Stats::Y() const
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
SignalStats Vector3Stats::Z() const
{
  return this->dataPtr->z;
}

//////////////////////////////////////////////////
SignalStats Vector3Stats::Mag() const
{
  return this->dataPtr->mag;
}

//////////////////////////////////////////////////
SignalStats &Vector3Stats::X()
{
  return this->dataPtr->x;
}

//////////////////////////////////////////////////
SignalStats &Vector3Stats::Y()
{
  return this->dataPtr->y;
}

//////////////////////////////////////////////////
SignalStats &Vector3Stats::Z()
{
  return this->dataPtr->z;
}

//////////////////////////////////////////////////
SignalStats &Vector3Stats::Mag()
{
  return this->dataPtr->mag;
}

