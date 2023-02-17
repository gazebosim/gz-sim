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
#ifndef GZ_SIM_SYSTEMS_ENVIRONMENTPRELOAD_VIZTOOL_HH_
#define GZ_SIM_SYSTEMS_ENVIRONMENTPRELOAD_VIZTOOL_HH_

#include <atomic>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/common/CSVStreams.hh>
#include <gz/common/DataFrame.hh>

#include <gz/transport/Node.hh>

#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/msgs/Utility.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
class EnvironmentVisualizationTool
{
  public: EnvironmentVisualizationTool()
  {
    this->pcPub =
      this->node.Advertise<gz::msgs::PointCloudPacked>("/point_cloud");
  }
  /// \brief To synchronize member access.
  public: std::mutex mutex;

  /// \brief first load we need to scan for existing data sensor
  public: bool first {true};

  public: std::atomic<bool> resample{true};

  /////////////////////////////////////////////////
  public: void CreatePointCloudTopics(
    std::shared_ptr<components::EnvironmentalData> data) {

    std::cout << __FILE__ << ": " << __LINE__ << std::endl;
    this->pubs.clear();
    this->sessions.clear();
    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    for (auto key : data->frame.Keys())
    {
          std::cout << __FILE__ << ": " << __LINE__ << std::endl;

      this->pubs.emplace(key, node.Advertise<gz::msgs::Float_V>(key));
      gz::msgs::Float_V msg;
      this->floatFields.emplace(key, msg);
          std::cout << __FILE__ << ": " << __LINE__ << std::endl;

      this->sessions.emplace(key, data->frame[key].CreateSession());
    }
  }

  /////////////////////////////////////////////////
  public: void Step(
    const UpdateInfo &_info,
    const EntityComponentManager& _ecm,
    double xSamples, double ySamples, double zSamples)
  {
    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt(now - this->lastTick);

    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    std::shared_ptr<components::EnvironmentalData> data;

    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    bool proceed{false};

    _ecm.Each<components::Environment>([&](
      const Entity &_e,
      const components::Environment *_env
    ) -> bool {
      if (_env)
      {
    //std::cout << __FILE__ << ": " << __LINE__ << std::endl;

        data = _env->Data();
        proceed = true;
    //std::cout << __FILE__ << ": " << __LINE__ << std::endl;

      }
    });

    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    if (!proceed)
      return;

    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    if (this->resample)
    {
    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

      this->CreatePointCloudTopics(data);


    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

      this->ResizeCloud(data, _ecm, xSamples, ySamples, zSamples);
      this->resample = false;
      this->lastTick = now;
        std::cout << __FILE__ << ": " << __LINE__ << std::endl;
  
    }

    std::cout << __FILE__ << ": " << __LINE__ << std::endl;

    for (auto &it : this->sessions)
    {
      auto res =
        data->frame[it.first].StepTo(it.second,
          std::chrono::duration<double>(_info.simTime).count());
      if (res.has_value())
      {
        it.second = res.value();
      }
    }
        std::cout << __FILE__ << ": " << __LINE__ << std::endl;


    // Publish at 2 hz for now. In future make reconfigureable.
    if (dt.count() > 0.5)
    {
      this->Visualize(data, xSamples, ySamples, zSamples);
      this->Publish();
      lastTick = now;
    }
  }

  /////////////////////////////////////////////////
  public: void Visualize(
    std::shared_ptr<components::EnvironmentalData> data,
    double xSamples, double ySamples, double zSamples) {

    for (auto key : data->frame.Keys())
    {
      const auto session = this->sessions[key];
      auto frame = data->frame[key];
      auto [lower_bound, upper_bound] = frame.Bounds(session);
      auto step = upper_bound - lower_bound;
      auto dx = step.X() / xSamples;
      auto dy = step.Y() / ySamples;
      auto dz = step.Z() / zSamples;
      std::size_t idx = 0;
      for (std::size_t x_steps = 0; x_steps < ceil(xSamples); x_steps++)
      {
        auto x = lower_bound.X() + x_steps * dx;
        for (std::size_t y_steps = 0; y_steps < ceil(ySamples); y_steps++)
        {
          auto y = lower_bound.Y() + y_steps * dy;
          for (std::size_t z_steps = 0; z_steps < ceil(zSamples); z_steps++)
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
  public: void Publish()
  {
    pcPub.Publish(this->pcMsg);
    for(auto &[key, pub] : this->pubs)
    {
      pub.Publish(this->floatFields[key]);
    }
  }

  /////////////////////////////////////////////////
  public: void ResizeCloud(
    std::shared_ptr<components::EnvironmentalData> data,
    const EntityComponentManager& _ecm,
    unsigned int xSamples, unsigned int ySamples, unsigned int zSamples)
  {
    assert (pubs.size() > 0);

    // Assume all data have same point cloud.
    gz::msgs::InitPointCloudPacked(pcMsg, "some_frame", true,
        {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});
    auto numberOfPoints = xSamples * ySamples * zSamples;
    std::size_t dataSize{
      static_cast<std::size_t>(numberOfPoints * pcMsg.point_step())};
    pcMsg.mutable_data()->resize(dataSize);
    pcMsg.set_height(1);
    pcMsg.set_width(numberOfPoints);

    auto session = this->sessions[this->pubs.begin()->first];
    auto frame = data->frame[this->pubs.begin()->first];
    auto [lower_bound, upper_bound] =
      frame.Bounds(session);

    auto step = upper_bound - lower_bound;
    auto dx = step.X() / xSamples;
    auto dy = step.Y() / ySamples;
    auto dz = step.Z() / zSamples;

    // Populate point cloud
    gz::msgs::PointCloudPackedIterator<float> xIter(pcMsg, "x");
    gz::msgs::PointCloudPackedIterator<float> yIter(pcMsg, "y");
    gz::msgs::PointCloudPackedIterator<float> zIter(pcMsg, "z");

    for (std::size_t x_steps = 0; x_steps < xSamples; x_steps++)
    {
      auto x = lower_bound.X() + x_steps * dx;
      for (std::size_t y_steps = 0; y_steps < ySamples; y_steps++)
      {
        auto y = lower_bound.Y() + y_steps * dy;
        for (std::size_t z_steps = 0; z_steps < zSamples; z_steps++)
        {
          auto z = lower_bound.Z() + z_steps * dz;
          auto coords = getGridFieldCoordinates(
            _ecm, math::Vector3d{x, y, z},
            data);

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
    for (auto key : data->frame.Keys())
    {
      this->floatFields[key].mutable_data()->Resize(
        numberOfPoints, std::nanf(""));
    }
  }

  public: transport::Node::Publisher pcPub;
  public: std::unordered_map<std::string, transport::Node::Publisher> pubs;
  public: std::unordered_map<std::string, gz::msgs::Float_V> floatFields;
  public: transport::Node node;
  public: gz::msgs::PointCloudPacked pcMsg;
  public: std::unordered_map<std::string,
    gz::math::InMemorySession<double, double>> sessions;
  public:  std::chrono::time_point<std::chrono::steady_clock> lastTick;
};
}
}
}
#endif
