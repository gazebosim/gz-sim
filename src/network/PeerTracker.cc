/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include "PeerTracker.hh"

#include <algorithm>
#include <utility>

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
PeerTracker::PeerTracker(
    PeerInfo _info,
    EventManager *_eventMgr,
    const transport::NodeOptions &_options):
  info(std::move(_info)),
  eventMgr(_eventMgr),
  node(_options)
{
  this->heartbeatPub =
      this->node.Advertise<private_msgs::PeerInfo>("heartbeat");
  this->announcePub =
      this->node.Advertise<private_msgs::PeerAnnounce>("announce");
  this->node.Subscribe("heartbeat", &PeerTracker::OnPeerHeartbeat, this);
  this->node.Subscribe("announce", &PeerTracker::OnPeerAnnounce, this);

  private_msgs::PeerAnnounce msg;
  *msg.mutable_info() = toProto(this->info);
  msg.set_state(private_msgs::PeerAnnounce::CONNECTING);
  this->announcePub.Publish(msg);

  this->heartbeatRunning = true;
  this->heartbeatThread = std::thread([this]()
    {
      this->HeartbeatLoop();
    });
}

/////////////////////////////////////////////////
PeerTracker::~PeerTracker()
{
  this->node.Unsubscribe("heartbeat");
  this->node.Unsubscribe("announce");

  this->heartbeatRunning = false;
  if (this->heartbeatThread.joinable())
  {
    this->heartbeatThread.join();
  }

  private_msgs::PeerAnnounce msg;
  *msg.mutable_info() = toProto(this->info);
  msg.set_state(private_msgs::PeerAnnounce::DISCONNECTING);

  this->announcePub.Publish(msg);
}

/////////////////////////////////////////////////
void PeerTracker::SetHeartbeatPeriod(const Duration &_period)
{
  this->heartbeatPeriod = _period;
}

/////////////////////////////////////////////////
PeerTracker::Duration PeerTracker::HeartbeatPeriod() const
{
  return this->heartbeatPeriod;
}

/////////////////////////////////////////////////
void PeerTracker::SetStaleMultiplier(const size_t &_multiplier)
{
  this->staleMultiplier = _multiplier;
}

/////////////////////////////////////////////////
size_t PeerTracker::StaleMultiplier() const
{
  return this->staleMultiplier;
}

/////////////////////////////////////////////////
size_t PeerTracker::NumPeers() const
{
  auto lock = PeerLock(this->peersMutex);
  return this->peers.size();
}

/////////////////////////////////////////////////
size_t PeerTracker::NumPeers(const NetworkRole &_role) const
{
  auto lock = PeerLock(this->peersMutex);

  size_t count = 0;
  for (auto peer : peers)
  {
    if (peer.second.info.role == _role)
    {
      count++;
    }
  }
  return count;
}

/////////////////////////////////////////////////
void PeerTracker::HeartbeatLoop()
{
  using Clock = std::chrono::steady_clock;
  Clock::time_point lastUpdateTime;

  while (this->heartbeatRunning)
  {
    lastUpdateTime = Clock::now();
    this->heartbeatPub.Publish(toProto(this->info));

    std::vector<PeerInfo> toRemove;
    for (auto peer : this->peers)
    {
      auto age = Clock::now() - peer.second.lastSeen;
      if (age > (this->staleMultiplier * this->heartbeatPeriod))
      {
        toRemove.push_back(peer.second.info);
      }
    }

    // Remove peers that were marked as stale.
    for (const auto &peer : toRemove)
    {
      this->OnPeerStale(peer);
    }

    // Compute sleep time to keep update loop as close to
    // heartbeatPeriod as possible.
    auto sleepTime = std::max(std::chrono::nanoseconds(0),
          lastUpdateTime + this->heartbeatPeriod - Clock::now());

    if (sleepTime > std::chrono::nanoseconds(0))
    {
      std::this_thread::sleep_for(sleepTime);
    }
  }
}

/////////////////////////////////////////////////
bool PeerTracker::RemovePeer(const PeerInfo &_info)
{
  auto lock = PeerLock(this->peersMutex);

  auto iter = this->peers.find(_info.id);
  if (iter == this->peers.end())
  {
    gzdbg << "Attempting to remove peer [" << _info.id << "] from ["
           << this->info.id << "] but it wasn't connected" << std::endl;
    return false;
  }

  this->peers.erase(iter);
  return true;
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerAnnounce(const private_msgs::PeerAnnounce &_announce)
{
  auto peer = fromProto(_announce.info());

  // Skip announcements from self.
  if (peer.id == this->info.id)
    return;

  switch (_announce.state())
  {
    case private_msgs::PeerAnnounce::CONNECTING:
      this->OnPeerAdded(peer);
      break;
    case private_msgs::PeerAnnounce::DISCONNECTING:
      this->OnPeerRemoved(peer);
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerHeartbeat(const private_msgs::PeerInfo &_info)
{
  auto peer = fromProto(_info);

  // Skip hearbeats from self.
  if (peer.id == this->info.id)
  {
    return;
  }

  auto lock = PeerLock(this->peersMutex);

  // If it doesn't exist, we may have missed a peer announce,
  // so add it here on the heartbeat.
  if (this->peers.find(peer.id) == this->peers.end())
  {
    this->OnPeerAdded(peer);
  }

  // Update information about the state of this peer.
  auto &peerState = this->peers[peer.id];
  peerState.lastSeen = std::chrono::steady_clock::now();
  peerState.lastHeader = std::chrono::steady_clock::time_point(
      std::chrono::seconds(_info.header().stamp().sec()) +
      std::chrono::nanoseconds(_info.header().stamp().nsec()));
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerAdded(const PeerInfo &_info)
{
  auto lock = PeerLock(this->peersMutex);
  auto peerState = PeerState();
  peerState.info = _info;
  peerState.lastSeen = std::chrono::steady_clock::now();
  this->peers[_info.id] = peerState;

  // Emit event for any consumers
  if (eventMgr)
    eventMgr->Emit<PeerAdded>(_info);
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerRemoved(const PeerInfo &_info)
{
  auto success = this->RemovePeer(_info);

  // Emit event for any consumers
  if (success && eventMgr)
    eventMgr->Emit<PeerRemoved>(_info);
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerStale(const PeerInfo &_info)
{
  auto success = this->RemovePeer(_info);

  // Emit event for any consumers
  if (success && eventMgr)
    eventMgr->Emit<PeerStale>(_info);
}
