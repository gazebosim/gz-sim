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

/*
 * Check README for detailed instructions.
 * Usage:
 *   $ gz sim -r worlds/lrauv_waypoint_demo.sdf
 *   $ ./lrauv_waypoint_control
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs.hh>
#include <gz/transport.hh>

using namespace gz;

// Generate waypoints in a sine wave
// and keep track of the nex point.
class wayPointTracker
{
  public :
    double xPos = 0;
    double yPos = 0;

    std::vector<std::vector<double>> wayPoints;
    int totalWaypoints = 0;
    int currentIndex = 0;

    // Generates points using:
    // x = A * cos (t + phi)
    // y = B * sin (t + phi)
    // t = 0 to Tmax, with deltaT sampling.
    void generateWaypoints(double A, double B,
        double Tmax, double deltaT,
        double xShift, double yShift)
    {
      for (double t = 0; t < Tmax; t = t + deltaT)
      {
        this->wayPoints.push_back(
            {A * std::cos(t) + xShift,
            B * std::sin(t) + yShift});
      }

      this->totalWaypoints = this->wayPoints.size();
    }

    // Returns whether the suppied x,y position is near the current waypoint,
    // and updates current index.
    bool arrivedAtWayPoint(double radius)
    {
      auto xWayPoint = this->wayPoints[this->currentIndex][0];
      auto yWayPoint = this->wayPoints[this->currentIndex][1];
      double distToWaypoint = std::pow(std::pow(this->xPos - xWayPoint, 2) +
        std::pow(this->yPos - yWayPoint, 2), 0.5);

      if (distToWaypoint < radius)
      {
        if (this->currentIndex < this->totalWaypoints - 1)
          this->currentIndex++;

        return true;
      }

      return false;
    }
};

int main(int argc, char** argv)
{
  auto tracker = wayPointTracker();

  transport::Node node;
  int linearSpeed = -10;

  // Propeller command publisher.
  auto propellerTopicTethys =
    gz::transport::TopicUtils::AsValidTopic(
      "/model/tethys/joint/propeller_joint/cmd_pos");
  auto propellerPubTethys =
    node.Advertise<gz::msgs::Double>(propellerTopicTethys);

  // Subscriber for vehicle pose.
  std::function<void(const msgs::Pose_V &)> cbPos =
    [&](const msgs::Pose_V &_msg)
  {
    tracker.xPos = _msg.pose()[0].position().x();
    tracker.yPos = _msg.pose()[0].position().y();
  };

  node.Subscribe("/model/tethys/pose", cbPos);

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Publish propeller command.
    msgs::Double propellerMsg;
    propellerMsg.set_data(linearSpeed);

    propellerPubTethys.Publish(propellerMsg);

    // Publish rudder msg.
    std::cout << "x, y: " << tracker.xPos << " " << tracker.yPos << std::endl;
  }
}
