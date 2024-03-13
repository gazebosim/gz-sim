/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "VisualizationTool.hh"

/////////////////////////////////////////////////
EnvironmentVisualizationTool::EnvironmentVisualizationTool()
{
  this->pcPub =
    this->node.Advertise<gz::msgs::PointCloudPacked>("/point_cloud");
}

/////////////////////////////////////////////////
void EnvironmentVisualizationTool::CreatePointCloudTopics(
    const std::shared_ptr<components::EnvironmentalData> &_data,
    const UpdateInfo &_info)
{
  this->pubs.clear();
  this->sessions.clear();

  for (auto key : _data->frame.Keys())
  {
    this->pubs.emplace(key, node.Advertise<gz::msgs::Float_V>(key));
    gz::msgs::Float_V msg;
    this->floatFields.emplace(key, msg);
    const double time = std::chrono::duration<double>(_info.simTime).count();
    auto sess = _data->frame[key].CreateSession(time);
    if (!_data->frame[key].IsValid(sess))
    {
      gzerr << key << "data is out of time bounds. Nothing will be published"
        <<std::endl;
      this->finishedTime = true;
      continue;
    }
    this->sessions.emplace(key, sess);
  }
}

/////////////////////////////////////////////////
void EnvironmentVisualizationTool::FileReloaded()
{
  this->finishedTime = false;
}

/////////////////////////////////////////////////
void EnvironmentVisualizationTool::Step(
    const UpdateInfo &_info,
    const EntityComponentManager &_ecm,
    const std::shared_ptr<components::EnvironmentalData> &_data,
    unsigned int _xSamples, unsigned int _ySamples, unsigned int _zSamples)
{
  if (this->finishedTime)
  {
    return;
  }
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> dt(now - this->lastTick);

  if (this->resample)
  {
    this->CreatePointCloudTopics(_data, _info);
    if (this->finishedTime) {
      this->resample = false;
      return;
    }
    this->ResizeCloud(_data, _ecm, _xSamples, _ySamples, _zSamples);
    this->resample = false;
    this->lastTick = now;
  }

  // Progress session pointers to next time stamp
  for (auto &it : this->sessions)
  {
    auto res =
      _data->frame[it.first].StepTo(it.second,
        std::chrono::duration<double>(_info.simTime).count());
    if (res.has_value())
    {
      it.second = res.value();
    }
    else
    {
      gzerr << "Data does not exist beyond this time."
        << " Not publishing new environment visuallization data."
        << std::endl;
      this->finishedTime = true;
      return;
    }
  }

  // Publish at 2 hz for now. In future make reconfigureable.
  if (dt.count() > 0.5 && !this->finishedTime)
  {
    this->Visualize(_data, _xSamples, _ySamples, _zSamples);
    this->Publish();
    lastTick = now;
  }
}

/////////////////////////////////////////////////
void EnvironmentVisualizationTool::Visualize(
    const std::shared_ptr<components::EnvironmentalData> &_data,
    unsigned int _xSamples, unsigned int _ySamples, unsigned int _zSamples)
{
  for (auto key : _data->frame.Keys())
  {
    const auto session = this->sessions[key];
    auto frame = _data->frame[key];
    auto [lower_bound, upper_bound] = frame.Bounds(session);
    auto step = upper_bound - lower_bound;
    auto dx = step.X() / _xSamples;
    auto dy = step.Y() / _ySamples;
    auto dz = step.Z() / _zSamples;
    std::size_t idx = 0;
    for (std::size_t x_steps = 0; x_steps < _xSamples; x_steps++)
    {
      auto x = lower_bound.X() + x_steps * dx;
      for (std::size_t y_steps = 0; y_steps < _ySamples; y_steps++)
      {
        auto y = lower_bound.Y() + y_steps * dy;
        for (std::size_t z_steps = 0; z_steps < _zSamples; z_steps++)
        {
          auto z = lower_bound.Z() + z_steps * dz;
          auto res = frame.LookUp(session, math::Vector3d(x, y, z));

          if (res.has_value())
          {
            this->floatFields[key].mutable_data()->Set(idx,
              static_cast<float>(res.value()));
          }
          else
          {
            this->floatFields[key].mutable_data()->Set(idx, std::nanf(""));
          }
          idx++;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void EnvironmentVisualizationTool::Publish()
{
  pcPub.Publish(this->pcMsg);
  for (auto &[key, pub] : this->pubs)
  {
    pub.Publish(this->floatFields[key]);
  }
}

/////////////////////////////////////////////////
void EnvironmentVisualizationTool::ResizeCloud(
  const std::shared_ptr<components::EnvironmentalData> &_data,
  const EntityComponentManager &_ecm,
  unsigned int _numXSamples,
  unsigned int _numYSamples,
  unsigned int _numZSamples)
{
  assert(pubs.size() > 0);

  // Assume all data have same point cloud.
  gz::msgs::InitPointCloudPacked(pcMsg, "some_frame", true,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});
  auto numberOfPoints = _numXSamples * _numYSamples * _numZSamples;
  std::size_t dataSize{
    static_cast<std::size_t>(numberOfPoints * pcMsg.point_step())};
  pcMsg.mutable_data()->resize(dataSize);
  pcMsg.set_height(1);
  pcMsg.set_width(numberOfPoints);

  auto session = this->sessions[this->pubs.begin()->first];
  auto frame = _data->frame[this->pubs.begin()->first];
  auto [lower_bound, upper_bound] = frame.Bounds(session);

  auto step = upper_bound - lower_bound;
  auto dx = step.X() / _numXSamples;
  auto dy = step.Y() / _numYSamples;
  auto dz = step.Z() / _numZSamples;

  // Populate point cloud
  gz::msgs::PointCloudPackedIterator<float> xIter(pcMsg, "x");
  gz::msgs::PointCloudPackedIterator<float> yIter(pcMsg, "y");
  gz::msgs::PointCloudPackedIterator<float> zIter(pcMsg, "z");

  for (std::size_t x_steps = 0; x_steps < _numXSamples; x_steps++)
  {
    auto x = lower_bound.X() + x_steps * dx;
    for (std::size_t y_steps = 0; y_steps < _numYSamples; y_steps++)
    {
      auto y = lower_bound.Y() + y_steps * dy;
      for (std::size_t z_steps = 0; z_steps < _numZSamples; z_steps++)
      {
        auto z = lower_bound.Z() + z_steps * dz;
        auto coords = getGridFieldCoordinates(_ecm, math::Vector3d{x, y, z},
          _data);

        if (!coords.has_value())
        {
          continue;
        }

        auto pos = coords.value();
        *xIter = pos.X();
        *yIter = pos.Y();
        *zIter = pos.Z();
        ++xIter;
        ++yIter;
        ++zIter;
      }
    }
  }
  for (auto key : _data->frame.Keys())
  {
    this->floatFields[key].mutable_data()->Resize(
      numberOfPoints, std::nanf(""));
  }
}
