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

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
PeerTracker::PeerTracker(
    EventManager* _eventMgr,
    const ignition::transport::NodeOptions &_options
    ):
  node(_options),
  heartbeat_period(std::chrono::milliseconds(100)),
  stale_multiplier(5),
  eventMgr(_eventMgr)
{
}

/////////////////////////////////////////////////
PeerTracker::~PeerTracker()
{
  Disconnect();
}

/////////////////////////////////////////////////
void PeerTracker::SetHeartbeatPeriod(const Duration &_period)
{
  this->heartbeat_period = _period;
}

/////////////////////////////////////////////////
PeerTracker::Duration PeerTracker::HeartbeatPeriod() const
{
  return this->heartbeat_period;
}

/////////////////////////////////////////////////
void PeerTracker::SetStaleMultiplier(const size_t &_multiplier)
{
  this->stale_multiplier = _multiplier;
}

/////////////////////////////////////////////////
size_t PeerTracker::StaleMultiplier() const
{
  return this->stale_multiplier;
}

/////////////////////////////////////////////////
void PeerTracker::Connect(std::shared_ptr<PeerInfo> _info)
{
  this->info = _info;

  this->heartbeat_pub = this->node.Advertise<msgs::PeerInfo>("heartbeat");
  this->announce_pub = this->node.Advertise<msgs::PeerAnnounce>("announce");
  this->node.Subscribe("heartbeat", &PeerTracker::OnPeerHeartbeat, this);
  this->node.Subscribe("announce", &PeerTracker::OnPeerAnnounce, this);

  msgs::PeerAnnounce msg;
  *msg.mutable_info() = ToProto(*this->info);
  msg.set_state(msgs::PeerAnnounce::CONNECTING);
  this->announce_pub.Publish(msg);

  this->heartbeat_running = true;
  this->heartbeat_thread = std::thread([this](){ this->HeartbeatLoop(); });
}

/////////////////////////////////////////////////
void PeerTracker::Disconnect()
{
  this->heartbeat_running = false;
  if (this->heartbeat_thread.joinable()) {
    this->heartbeat_thread.join();
  }

  if (this->info)
  {
    msgs::PeerAnnounce msg;
    *msg.mutable_info() = ToProto(*this->info);
    msg.set_state(msgs::PeerAnnounce::DISCONNECTING);

    this->announce_pub.Publish(msg);
    this->info.reset();
  }

  this->node.Unsubscribe("heartbeat");
  this->node.Unsubscribe("announce");
}

/////////////////////////////////////////////////
size_t PeerTracker::NumPeers() const
{
  auto lock = PeerLock(this->peers_mutex);
  return this->peers.size();
}

/////////////////////////////////////////////////
size_t PeerTracker::NumPeers(const NetworkRole &_role) const
{
  auto lock = PeerLock(this->peers_mutex);

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

  while (this->heartbeat_running)
  {
    lastUpdateTime = Clock::now();
    this->heartbeat_pub.Publish(ToProto(*this->info));

    std::vector<PeerInfo> toRemove;
    for (auto peer : peers)
    {
      auto age = Clock::now() - peer.second.last_seen;
      if (age > (this->stale_multiplier * this->heartbeat_period))
      {
        toRemove.push_back(peer.second.info);
      }
    }

    // Remove peers that were marked as stale.
    for (auto peer : toRemove)
    {
      this->OnPeerStale(peer);
    }

    // Compute sleep time to keep update loop as close to
    // heartbeatPeriod as possible.
    auto sleepTime = std::max(std::chrono::nanoseconds(0),
          lastUpdateTime + this->heartbeat_period - Clock::now());

    if (sleepTime > std::chrono::nanoseconds(0))
    {
      std::this_thread::sleep_for(sleepTime);
    }
  }
}

/////////////////////////////////////////////////
void PeerTracker::AddPeer(const PeerInfo &_info)
{
  auto lock = PeerLock(this->peers_mutex);

  auto peerState = PeerState();
  peerState.info = _info;
  peerState.last_seen = std::chrono::steady_clock::now();
  this->peers[_info.id] = peerState;
}

/////////////////////////////////////////////////
void PeerTracker::RemovePeer(const PeerInfo &_info)
{
  auto lock = PeerLock(this->peers_mutex);

  auto iter = this->peers.find(_info.id);
  if (iter == this->peers.end())
  {
    igndbg << "Atempting to remove a peer that wasn't connected" << std::endl;
  }
  else
  {
    this->peers.erase(iter);
  }
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerAnnounce(const msgs::PeerAnnounce &_announce)
{
  auto peer = FromProto(_announce.info());

  // Skip announcements from self.
  if (this->info && peer.id == this->info->id)
    return;

  switch (_announce.state()) {
    case msgs::PeerAnnounce::CONNECTING:
      this->OnPeerAdded(peer);
      break;
    case msgs::PeerAnnounce::DISCONNECTING:
      this->OnPeerRemoved(peer);
      break;
    case msgs::PeerAnnounce::ERROR:
      this->OnPeerError(peer);
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerHeartbeat(const msgs::PeerInfo &_info)
{
  auto peer = FromProto(_info);

  if (!this->info)
  {
    return;
  }

  // Skip hearbeats from self.
  if (peer.id == this->info->id)
  {
    return;
  }

  auto lock = PeerLock(this->peers_mutex);

  // We may have missed a peer announce, add it on heartbeat.
  if (this->peers.find(peer.id) == this->peers.end())
  {
    this->OnPeerAdded(peer);
  }

  // Update information about the state of this peer.
  auto& peerState = this->peers[peer.id];
  peerState.last_seen = std::chrono::steady_clock::now();
  peerState.last_header = std::chrono::steady_clock::time_point(
      std::chrono::seconds(_info.header().stamp().sec()) +
      std::chrono::nanoseconds(_info.header().stamp().nsec()));
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerError(const PeerInfo &_info)
{
  this->RemovePeer(_info);

  // Emit event for any consumers
  if (eventMgr)
    eventMgr->Emit<PeerError>(_info);
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerAdded(const PeerInfo &_info)
{
  this->AddPeer(_info);

  // Emit event for any consumers
  if (eventMgr)
    eventMgr->Emit<PeerAdded>(_info);
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerRemoved(const PeerInfo &_info)
{
  this->RemovePeer(_info);

  // Emit event for any consumers
  if (eventMgr)
    eventMgr->Emit<PeerRemoved>(_info);
}

/////////////////////////////////////////////////
void PeerTracker::OnPeerStale(const PeerInfo &_info)
{
  this->RemovePeer(_info);

  // Emit event for any consumers
  if (eventMgr)
    eventMgr->Emit<PeerStale>(_info);
}
