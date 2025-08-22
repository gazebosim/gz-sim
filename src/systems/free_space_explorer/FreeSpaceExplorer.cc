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
#include "FreeSpaceExplorer.hh"

#include <gz/common/Image.hh>
#include <gz/math/OccupancyGrid.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>
#include <string>

using namespace gz;
using namespace sim;
using namespace systems;

// Custom hash function for std::pair<int, int>
struct PairHash {
  template <class T1, class T2>
  std::size_t operator () (const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);

    // Simple way to combine hashes. You might want a more robust one
    // for very specific use cases, but this is generally sufficient.
    return h1 ^ (h2 << 1); // XOR with a left shift to mix bits
  }
};

struct gz::sim::systems::FreeSpaceExplorerPrivateData {
  std::optional<math::OccupancyGrid> grid;
  Model model;
  std::size_t numRows, numCols;
  double resolution;
  std::string scanTopic;
  std::string sensorLink;
  math::Pose3d position;
  gz::transport::Node node;

  bool recievedMessageForPose {false};
  std::queue<math::Pose3d> nextPosition;
  std::unordered_set<std::pair<int,int>, PairHash> previouslyVisited;

  std::recursive_mutex m;

  /////////////////////////////////////////////////
  /// \brief Perform search over occupancy grid to see if there are any
  /// reachable unknown cells and return their count.
  int CountReachableUnknowns()
  {
    const std::lock_guard<std::recursive_mutex> lock(this->m);
    if (!this->grid.has_value())
    {
      gzerr << "Grid not yet inited" << std::endl;
      return 0;
    }
    std::queue<std::pair<int, int>> q;
    std::unordered_set<std::pair<int, int>, PairHash> visited;

    int gridX, gridY;
    if(!this->grid->WorldToGrid( this->position.Pos().X(),
      this->position.Pos().Y(), gridX, gridY))
    {
      gzerr << "Current position outside of bounds" << std::endl;
      return 0;
    }
    q.emplace(gridX, gridY);
    visited.emplace(gridX, gridY);

    int unknownCellCount = 0;
    while (!q.empty())
    {
      auto pt = q.front();
      q.pop();

      for(int i = -1; i <=1; i++)
      {
        for(int j = -1; j <=1; j++)
        {
          if (i == 0 && j == 0)
            continue;

          auto x = pt.first + i;
          auto y = pt.second + j;

          auto neighbor = std::make_pair(x, y);
          if(visited.count(neighbor) > 0)
          {
            continue;
          }

          auto cellState = this->grid->CellState(x, y);
          if (cellState == math::OccupancyCellState::Free)
          {
            q.emplace(neighbor);
          }
          else if (cellState == math::OccupancyCellState::Unknown)
          {
            unknownCellCount++;
          }
          visited.emplace(neighbor);
        }
      }
    }
    return unknownCellCount;
  }

  /////////////////////////////////////////////////
  /// Callback for laser scan message
  void OnLaserScanMsg(const msgs::LaserScan &_scan)
  {
    const std::lock_guard<std::recursive_mutex> lock(this->m);
    if (!this->grid.has_value())
    {
      gzerr<< "Grid not yet inited";
      return;
    }

    double currAngle = _scan.angle_min();

    /// Iterate through laser scan and mark bressenham line of free space
    for (uint32_t index = 0; index < _scan.count(); index++)
    {
      auto length = _scan.ranges(index);
      auto obstacleExists = length <= _scan.range_max();
      length = (length > _scan.range_max()) ? _scan.range_max() : length;
      auto toX = length * cos(currAngle) + this->position.Pos().X();
      auto toY = length * sin(currAngle) + this->position.Pos().Y();

      this->grid->MarkFree(this->position.Pos().X(), this->position.Pos().Y(), toX, toY);

      if (obstacleExists)
        this->grid->MarkOccupied(toX, toY);

      currAngle += _scan.angle_step();
    }

    if (!this->nextPosition.empty())
    {
      return;
    }

    gzerr << this->CountReachableUnknowns() << "\n";

    if (this->CountReachableUnknowns() == 0)
    {
      std::vector<unsigned char> pixelData;
      this->grid->ExportToRGBImage(pixelData);
      common::Image fromOccupancy;
      fromOccupancy.SetFromData(
      pixelData.data(), this->grid->Width(), this->grid->Height(), common::Image::PixelFormatType::RGB_INT8);
        fromOccupancy.SavePNG("output.png");
      gzmsg << "Scan complete: No reachable unknown cells.\n";
      return;
    }

    auto nextPos = this->GetNextPoint(_scan);
    if(nextPos.has_value())
    {
      gzmsg << "Setting next position " << nextPos->Pos() <<std::endl;
      this->nextPosition.push(nextPos.value());
    }
    else {
      std::vector<unsigned char> pixelData;
      this->grid->ExportToRGBImage(pixelData);
      common::Image fromOccupancy;
      fromOccupancy.SetFromData(
      pixelData.data(), this->grid->Width(), this->grid->Height(), common::Image::PixelFormatType::RGB_INT8);
        fromOccupancy.SavePNG("output.png");
      gzmsg << "Scan complete\n";
    }
  }

  /////////////////////////////////////////////////
  /// Perform search over occupancy grid for next position to
  /// explore.
  std::optional<math::Pose3d> GetNextPoint(const msgs::LaserScan &_scan)
  {
    const std::lock_guard<std::recursive_mutex> lock(this->m);
    if (!this->grid.has_value())
    {
      gzerr << "Grid not yet inited" << std::endl;
      return {};
    }
    std::queue<std::pair<int, int>> q;
    std::unordered_set<std::pair<int, int>, PairHash> visited;

    int gridX, gridY;
    if(!this->grid->WorldToGrid( this->position.Pos().X(),
      this->position.Pos().Y(), gridX, gridY))
    {
      gzerr << "Proposed point outside of bounds" << std::endl;
      return {};
    }
    q.emplace(gridX, gridY);
    visited.emplace(gridX, gridY);
    auto maxInfoGain = 0;
    std::pair<int, int> bestGain =
      std::make_pair(gridX, gridY);

    auto numPoints = 0;

    while (!q.empty())
    {
      auto pt = q.front();
      q.pop();

      for(int i = -1; i <=1; i++)
      {
        for(int j = -1; j <=1; j++)
        {

          auto x = pt.first + i;
          auto y = pt.second + j;

          if(visited.count(std::make_pair(x,y)) > 0)
          {
            continue;
          }
          numPoints++;

          if(this->grid->CellState(x, y) == math::OccupancyCellState::Free)
          {

            auto infoGain = this->ScoreInfoGain(x, y, _scan);
            if (infoGain.has_value() && infoGain > maxInfoGain
              && previouslyVisited.count(std::make_pair(x, y)) == 0)
            {
              bestGain = std::make_pair(x, y);
              maxInfoGain = infoGain.value();
            }

            visited.emplace(x, y);
            q.emplace(x, y);
          }
        }
      }
    }
    gzerr << "Visited " << visited.size() << "\n";
    gzerr << "Infogan" << maxInfoGain << "\n";
    if (maxInfoGain < 1)
    {
      gzmsg << "Could not find areas of information gain\n";
      gzmsg << maxInfoGain << "\n";
      return std::nullopt;
    }

    previouslyVisited.emplace(bestGain);

    double newX, newY;
    this->grid->GridToWorld(bestGain.first, bestGain.second, newX, newY);
    math::Pose3d newPose(this->position);
    newPose.Pos().X(newX);
    newPose.Pos().Y(newY);
    newPose.Pos().Z(this->position.Pos().Z());
    return newPose;
  }

  /// Scores information gain given laser scan parameters
  std::optional<double> ScoreInfoGain(int _x, int _y, const msgs::LaserScan &_scan)
  {
    const std::lock_guard<std::recursive_mutex> lock(this->m);
    if (!this->grid.has_value())
    {
      gzerr<< "Waiting for occupancy grid to be initialized." << std::endl;
      return {};
    }

    double currAngle = _scan.angle_min();
    auto numCells = _scan.range_max() / this->resolution;

    double infoGain = 0.0;

    /// Iterate through laser scan and evaluate information gain
    for (uint32_t index = 0; index < _scan.count(); index++)
    {
      auto x = static_cast<int>(std::round(numCells * cos(currAngle) + _x));
      auto y = static_cast<int>(std::round(numCells * sin(currAngle) + _y));
      infoGain +=this->grid->CalculateIGain(_x, _y, x, y);

      currAngle += _scan.angle_step();
    }

    return infoGain;
  }
};

/////////////////////////////////////////////////
FreeSpaceExplorer::FreeSpaceExplorer()
{

  this->dataPtr = std::make_unique<FreeSpaceExplorerPrivateData>();
}

/////////////////////////////////////////////////
FreeSpaceExplorer::~FreeSpaceExplorer()
{
}

/////////////////////////////////////////////////
void FreeSpaceExplorer::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &/*_ecm*/,
  gz::sim::EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  this->dataPtr->scanTopic = _sdf->Get<std::string>("lidar_topic", "scan").first;
  this->dataPtr->sensorLink = _sdf->Get<std::string>("sensor_link", "link").first;
  this->dataPtr->numRows = _sdf->Get<std::size_t>("width", 10).first;
  this->dataPtr->numCols = _sdf->Get<std::size_t>("height", 10).first;
  this->dataPtr->resolution = _sdf->Get<double>("resolution", 1.0).first;
  this->dataPtr->node.Subscribe(this->dataPtr->scanTopic,
    &FreeSpaceExplorerPrivateData::OnLaserScanMsg, this->dataPtr.get());
  gzmsg << "Loaded camera plugin listening on [" << this->dataPtr->scanTopic << "]\n";
}

/////////////////////////////////////////////////
void FreeSpaceExplorer::PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }
  //TODO(arjo) check link name valisdity
  auto l = Link(this->dataPtr->model.LinkByName(_ecm, this->dataPtr->sensorLink));
  if (!l.Valid(_ecm)){
    gzerr << "Invalid link name " << this->dataPtr->sensorLink << std::endl;
    return;
  }
  auto pose = l.WorldPose(_ecm);
  if (!pose.has_value()) {
    l.EnableVelocityChecks(_ecm);
    return;
  }

  const std::lock_guard<std::recursive_mutex> lock(this->dataPtr->m);
  if (!this->dataPtr->grid.has_value())
  {
    auto center_x = pose->Pos().X() - this->dataPtr->numRows * this->dataPtr->resolution / 2;
    auto center_y = pose->Pos().Y() - this->dataPtr->numCols * this->dataPtr->resolution / 2;
    math::OccupancyGrid g(
      this->dataPtr->resolution, this->dataPtr->numRows, this->dataPtr->numCols,
      center_x, center_y);
    this->dataPtr->grid = {std::move(g)};
    this->dataPtr->position = pose.value();
  }
  this->dataPtr->position = pose.value();

  if (this->dataPtr->nextPosition.empty())
  {
    return;
  }

  auto modelPosCmd = this->dataPtr->nextPosition.front();
  this->dataPtr->nextPosition.pop();
  this->dataPtr->model.SetWorldPoseCmd(_ecm, modelPosCmd);
}

GZ_ADD_PLUGIN(
    FreeSpaceExplorer,
    gz::sim::System,
    FreeSpaceExplorer::ISystemPreUpdate,
    FreeSpaceExplorer::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
  FreeSpaceExplorer,
  "gz::sim::systems::FreeSpaceExplorer")
