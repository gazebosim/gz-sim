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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>

#include <atomic>
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
    return h1 ^ (h2 << 1);
  }
};

/// \brief Private data pointer that performs actual
/// exploration.
struct gz::sim::systems::FreeSpaceExplorerPrivateData {

  /// \brief Occupancy Grid function
  std::optional<math::OccupancyGrid> grid;

  /// \brief Model of the parent sensor that gets
  /// moved around.
  Model model;

  /// \brief Number of rows and columns the occupancy grid should
  /// have.
  std::size_t numRows, numCols;

  /// \brief Resolution parameter for the occupancy grid
  double resolution;

  /// \brief Image publisher for preview image of occupancy grid
  gz::transport::Node::Publisher imagePub;

  /// \brief Sensor link
  std::string sensorLink;

  /// \brief The starting position of the occupancy grid.
  math::Pose3d position;

  /// \brief GZ-Transport node
  gz::transport::Node node;

  /// \brief The signal to enable exploration
  std::atomic<bool> explorationStarted {false};

  /// \brief The next position to move the sensor to increase
  /// coverage.
  std::queue<math::Pose3d> nextPosition;

  /// \brief Previously visited locations
  std::unordered_set<std::pair<int, int>, PairHash>
    previouslyVisited;

  /// \brief Mutex to protect data in this structure.
  std::recursive_mutex m;

  /// \brief Laser scan message queue
  std::queue<gz::msgs::LaserScan> laserScanMsgs;

  /// \brief Mutex for the laser scan queue
  std::mutex laserScanMutex;

  /////////////////////////////////////////////////
  /// \brief Callback for start message
  void OnStartMsg(const msgs::Boolean &_scan)
  {
    if (_scan.data())
    {
      this->explorationStarted = true;
    }
  }

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
    std::lock_guard<std::mutex> lock(this->laserScanMutex);
    this->laserScanMsgs.push(_scan);
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
      gzerr << "Proposed point outside of bounds" <<std::endl;
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

          if(visited.count(std::make_pair(x, y)) > 0)
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

    if (maxInfoGain < 1)
    {
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
  std::optional<double> ScoreInfoGain(int _x, int _y,
    const msgs::LaserScan &_scan)
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
  auto scanTopic =
    _sdf->Get<std::string>("lidar_topic", "scan").first;
  auto imageTopic =
    _sdf->Get<std::string>("image_topic", "scan_image").first;
  auto startTopic =
    _sdf->Get<std::string>("start_topic", "start").first;
  this->dataPtr->sensorLink =
    _sdf->Get<std::string>("sensor_link", "link").first;
  this->dataPtr->numRows =
    _sdf->Get<std::size_t>("width", 10).first;
  this->dataPtr->numCols =
    _sdf->Get<std::size_t>("height", 10).first;
  this->dataPtr->resolution =
    _sdf->Get<double>("resolution", 1.0).first;
  this->dataPtr->node.Subscribe(scanTopic,
    &FreeSpaceExplorerPrivateData::OnLaserScanMsg, this->dataPtr.get());
  this->dataPtr->node.Subscribe(startTopic,
    &FreeSpaceExplorerPrivateData::OnStartMsg, this->dataPtr.get());
  this->dataPtr->imagePub =
  this->dataPtr->node.Advertise<gz::msgs::Image>(imageTopic);
  gzmsg << "Loaded lidar exploration plugin listening on ["
    << scanTopic << "] for lidar messages\n";
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
  auto link =
    Link(this->dataPtr->model.LinkByName(_ecm, this->dataPtr->sensorLink));
  if (!link.Valid(_ecm)){
    gzerr << "Invalid link name " << this->dataPtr->sensorLink << std::endl;
    return;
  }
  auto pose = link.WorldPose(_ecm);
  if (!pose.has_value()) {
    link.EnableVelocityChecks(_ecm);
    return;
  }

  const std::lock_guard<std::recursive_mutex> lock(this->dataPtr->m);
  if (!this->dataPtr->grid.has_value())
  {
    auto centerX =
      pose->Pos().X() - this->dataPtr->numRows * this->dataPtr->resolution / 2;
    auto centerY =
      pose->Pos().Y() - this->dataPtr->numCols * this->dataPtr->resolution / 2;
    math::OccupancyGrid g(
      this->dataPtr->resolution, this->dataPtr->numRows, this->dataPtr->numCols,
      centerX, centerY);
    this->dataPtr->grid = {std::move(g)};
    this->dataPtr->position = pose.value();
  }
  this->dataPtr->position = pose.value();

  if (this->dataPtr->nextPosition.empty() || !this->dataPtr->explorationStarted)
  {
    return;
  }

  auto modelPosCmd = this->dataPtr->nextPosition.front();
  this->dataPtr->nextPosition.pop();
  this->dataPtr->model.SetWorldPoseCmd(_ecm, modelPosCmd);
}

/////////////////////////////////////////////////
void FreeSpaceExplorer::PostUpdate(
  const gz::sim::UpdateInfo &_info,
  const gz::sim::EntityComponentManager &/*_ecm*/)
{
  if (_info.paused)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->laserScanMutex);
  if (this->dataPtr->laserScanMsgs.empty())
  {
    return;
  }

  auto scan = this->dataPtr->laserScanMsgs.front();
  this->dataPtr->laserScanMsgs.pop();

  const std::lock_guard<std::recursive_mutex> lock2(this->dataPtr->m);
  if (!this->dataPtr->grid.has_value())
  {
    gzerr<< "Grid not yet inited";
    return;
  }

  if (!this->dataPtr->explorationStarted)
  {
    return;
  }

  double currAngle = scan.angle_min();

  /// Iterate through laser scan and mark bressenham line of free space
  for (uint32_t index = 0; index < scan.count(); index++)
  {
    auto length = scan.ranges(index);
    auto obstacleExists = length <= scan.range_max();
    length = (length > scan.range_max()) ? scan.range_max() : length;
    auto toX = length * cos(currAngle) + this->dataPtr->position.Pos().X();
    auto toY = length * sin(currAngle) + this->dataPtr->position.Pos().Y();

    this->dataPtr->grid->MarkFree(this->dataPtr->position.Pos().X(),
      this->dataPtr->position.Pos().Y(), toX, toY);

    if (obstacleExists)
      this->dataPtr->grid->MarkOccupied(toX, toY);

    currAngle += scan.angle_step();
  }

  if (!this->dataPtr->nextPosition.empty())
  {
    return;
  }

  if (this->dataPtr->CountReachableUnknowns() == 0)
  {
    std::vector<unsigned char> pixelData;
    this->dataPtr->grid->ExportToRGBImage(pixelData);
    gz::msgs::Image imageMsg;
    imageMsg.set_width(this->dataPtr->grid->Width());
    imageMsg.set_height(this->dataPtr->grid->Height());
    imageMsg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
    imageMsg.set_step(this->dataPtr->grid->Width() * 3);
    imageMsg.set_data(pixelData.data(), pixelData.size());
    this->dataPtr->imagePub.Publish(imageMsg);
    gzmsg << "Scan complete: No reachable unknown cells.\n";
    return;
  }

  auto nextPos = this->dataPtr->GetNextPoint(scan);
  if(nextPos.has_value())
  {
    gzmsg << "Setting next position " << nextPos->Pos() <<std::endl;
    this->dataPtr->nextPosition.push(nextPos.value());
  }

  std::vector<unsigned char> pixelData;
  this->dataPtr->grid->ExportToRGBImage(pixelData);
  gz::msgs::Image imageMsg;
  imageMsg.set_width(this->dataPtr->grid->Width());
  imageMsg.set_height(this->dataPtr->grid->Height());
  imageMsg.set_pixel_format_type(gz::msgs::PixelFormatType::RGB_INT8);
  imageMsg.set_step(this->dataPtr->grid->Width() * 3);
  imageMsg.set_data(pixelData.data(), pixelData.size());
  this->dataPtr->imagePub.Publish(imageMsg);
}


GZ_ADD_PLUGIN(
    FreeSpaceExplorer,
    gz::sim::System,
    FreeSpaceExplorer::ISystemPreUpdate,
    FreeSpaceExplorer::ISystemPostUpdate,
    FreeSpaceExplorer::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(
  FreeSpaceExplorer,
  "gz::sim::systems::FreeSpaceExplorer")
