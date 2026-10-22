/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include "TimelinePublisher.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <google/protobuf/message.h>

#include <gz/common/Profiler.hh>
#include <gz/msgs/Factory.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>
#include <sdf/Element.hh>

using namespace gz;
using namespace sim;
using namespace systems;

namespace
{
using Duration = std::chrono::steady_clock::duration;

/// \brief A message published by a keyframe.
struct Output
{
  /// \brief Publisher for the message. Outputs on the same topic share it.
  transport::Node::Publisher pub;

  /// \brief Message to publish.
  std::unique_ptr<google::protobuf::Message> msg;
};

/// \brief A topic advertised by the timeline.
struct Topic
{
  /// \brief Message type of the topic.
  std::string type;

  /// \brief Publisher for the topic.
  transport::Node::Publisher pub;
};

/// \brief A service called by a keyframe.
struct ServiceCall
{
  /// \brief Service name.
  std::string name;

  /// \brief Request message.
  std::unique_ptr<google::protobuf::Message> req;

  /// \brief Empty response message, copied for each call.
  std::unique_ptr<google::protobuf::Message> rep;

  /// \brief Timeout in milliseconds.
  unsigned int timeout{1000};
};

/// \brief A point in the timeline.
struct Keyframe
{
  /// \brief Time since the start of the timeline.
  Duration time{0};

  /// \brief Messages to publish.
  std::vector<Output> outputs;

  /// \brief Services to call.
  std::vector<ServiceCall> services;
};

/// \brief A service call waiting for the worker thread.
struct PendingCall
{
  /// \brief Call to make.
  const ServiceCall *call{nullptr};

  /// \brief Cycle of the timeline that queued the call.
  std::size_t cycle{0};
};

/// \brief Convert seconds to a duration, rounding to the nearest tick so
/// that a time like 0.3 s isn't truncated to one tick earlier.
/// \param[in] _seconds Time in seconds.
/// \return The duration, or nullopt if the time is negative, not finite or
/// too large for a duration.
std::optional<Duration> ToDuration(double _seconds)
{
  const std::chrono::duration<double> seconds(_seconds);
  if (!std::isfinite(_seconds) || _seconds < 0.0 ||
      seconds >= std::chrono::duration<double>(Duration::max()))
  {
    return std::nullopt;
  }
  return std::chrono::round<Duration>(seconds);
}

/// \brief Parse a service element.
/// \param[in] _elem The <service> element.
/// \return The service call, or nullopt if the element is invalid.
std::optional<ServiceCall> ParseService(const sdf::ElementConstPtr &_elem)
{
  ServiceCall call;
  call.name = transport::TopicUtils::AsValidTopic(
      _elem->Get<std::string>("name"));
  const auto reqType = _elem->Get<std::string>("reqType");
  const auto repType = _elem->Get<std::string>("repType");
  call.timeout = _elem->Get<unsigned int>("timeout", 1000u).first;
  if (call.name.empty() || reqType.empty() || repType.empty())
  {
    gzerr << "A service needs valid [name], [reqType] and [repType] "
          << "attributes. Ignoring it.\n";
    return std::nullopt;
  }

  // A long request reads better as the content of the element.
  call.req = msgs::Factory::New(reqType, _elem->HasAttribute("reqMsg") ?
      _elem->Get<std::string>("reqMsg") : _elem->Get<std::string>());
  call.rep = msgs::Factory::New(repType);
  if (!call.req || !call.rep)
  {
    gzerr << "Unable to create the request [" << reqType << "] or response ["
          << repType << "] for service [" << call.name << "]. Ignoring it.\n";
    return std::nullopt;
  }
  return call;
}
}  // namespace

class gz::sim::systems::TimelinePublisherPrivate
{
  /// \brief Destructor. Stops the service worker thread.
  public: ~TimelinePublisherPrivate();

  /// \brief Parse an output element.
  /// \param[in] _elem The <output> element.
  /// \param[in, out] _topics Topics advertised so far. A node can only
  /// advertise a topic once, so outputs on the same topic share a publisher.
  /// \return The output, or nullopt if the element is invalid.
  public: std::optional<Output> ParseOutput(const sdf::ElementConstPtr &_elem,
      std::unordered_map<std::string, Topic> &_topics);

  /// \brief Start the timeline over on the next update and drop the service
  /// calls still waiting, which belong to the previous run.
  public: void Restart();

  /// \brief Fire the keyframes that are due at a simulation time.
  /// \param[in] _simTime Current simulation time.
  public: void Update(const Duration &_simTime);

  /// \brief Publish the outputs of a keyframe and queue its service calls.
  /// \param[in] _keyframe Keyframe to fire.
  public: void Fire(Keyframe &_keyframe);

  /// \brief Drop the service calls still waiting from cycles before the
  /// previous one. Calls from the previous cycle are kept, because its last
  /// keyframe can fire on the same step that the current cycle starts.
  public: void DropStaleServices();

  /// \brief Call queued services until the system is destroyed.
  public: void ServiceWorker();

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Keyframes sorted by time. Their service calls don't change after
  /// Configure, so the worker thread reads them without locking.
  public: std::vector<Keyframe> keyframes;

  /// \brief Whether the timeline loops.
  public: bool loop{false};

  /// \brief Length of the timeline when it loops. It is at least the time of
  /// the last keyframe.
  public: Duration period{0};

  /// \brief Simulation time at which the current cycle started. Unset until
  /// the first update and after a reset.
  public: std::optional<Duration> cycleStart;

  /// \brief Simulation time of the last update.
  public: Duration lastSimTime{0};

  /// \brief Index of the next keyframe to fire in the current cycle.
  public: std::size_t next{0};

  /// \brief Number of the current cycle, counted from zero.
  public: std::size_t cycle{0};

  /// \brief Whether a warning about dropped service calls was printed.
  public: bool droppedWarned{false};

  /// \brief Service calls waiting for the worker thread, oldest first.
  public: std::deque<PendingCall> pendingServices;

  /// \brief Protects pendingServices and done.
  public: std::mutex serviceMutex;

  /// \brief Wakes up the worker thread.
  public: std::condition_variable serviceCv;

  /// \brief Set to stop the worker thread.
  public: bool done{false};

  /// \brief Thread that calls services. Only started if a keyframe calls a
  /// service.
  public: std::thread serviceThread;
};

//////////////////////////////////////////////////
TimelinePublisher::TimelinePublisher()
  : dataPtr(std::make_unique<TimelinePublisherPrivate>())
{
}

//////////////////////////////////////////////////
TimelinePublisher::~TimelinePublisher() = default;

//////////////////////////////////////////////////
TimelinePublisherPrivate::~TimelinePublisherPrivate()
{
  if (!this->serviceThread.joinable())
    return;

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    this->done = true;
  }
  this->serviceCv.notify_all();
  this->serviceThread.join();
}

//////////////////////////////////////////////////
std::optional<Output> TimelinePublisherPrivate::ParseOutput(
    const sdf::ElementConstPtr &_elem,
    std::unordered_map<std::string, Topic> &_topics)
{
  const auto topic = transport::TopicUtils::AsValidTopic(
      _elem->Get<std::string>("topic"));
  const auto type = _elem->Get<std::string>("type");
  if (topic.empty() || type.empty())
  {
    gzerr << "An output needs valid [topic] and [type] attributes. "
          << "Ignoring it.\n";
    return std::nullopt;
  }

  Output output;
  output.msg = msgs::Factory::New(type, _elem->Get<std::string>());
  if (!output.msg)
  {
    gzerr << "Unable to create a message of type [" << type
          << "] for topic [" << topic << "]. Ignoring it.\n";
    return std::nullopt;
  }

  auto it = _topics.find(topic);
  if (it == _topics.end())
  {
    auto pub = this->node.Advertise(topic, type);
    if (!pub)
    {
      gzerr << "Unable to advertise topic [" << topic << "] with type ["
            << type << "]. Ignoring the output.\n";
      return std::nullopt;
    }
    it = _topics.emplace(topic, Topic{type, std::move(pub)}).first;
  }
  else if (it->second.type != type)
  {
    gzerr << "Topic [" << topic << "] is already used with type ["
          << it->second.type << "], it can't also be used with type ["
          << type << "]. Ignoring the output.\n";
    return std::nullopt;
  }
  output.pub = it->second.pub;
  return output;
}

//////////////////////////////////////////////////
void TimelinePublisher::Configure(const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  auto &keyframes = this->dataPtr->keyframes;
  std::unordered_map<std::string, Topic> topics;
  for (auto kfElem = _sdf->FindElement("keyframe"); kfElem;
       kfElem = kfElem->GetNextElement("keyframe"))
  {
    auto timeAttr = kfElem->GetAttribute("time");
    double seconds{-1.0};
    std::optional<Duration> time;
    if (timeAttr && timeAttr->Get(seconds))
      time = ToDuration(seconds);
    if (!time)
    {
      gzerr << "Keyframe without a valid non negative [time] attribute. "
            << "Ignoring it.\n";
      continue;
    }

    Keyframe keyframe;
    keyframe.time = *time;
    for (auto elem = kfElem->FindElement("output"); elem;
         elem = elem->GetNextElement("output"))
    {
      if (auto output = this->dataPtr->ParseOutput(elem, topics))
        keyframe.outputs.push_back(std::move(*output));
    }
    for (auto elem = kfElem->FindElement("service"); elem;
         elem = elem->GetNextElement("service"))
    {
      if (auto call = ParseService(elem))
        keyframe.services.push_back(std::move(*call));
    }
    keyframes.push_back(std::move(keyframe));
  }

  if (keyframes.empty())
  {
    gzwarn << "TimelinePublisher has no keyframes.\n";
    return;
  }

  std::stable_sort(keyframes.begin(), keyframes.end(),
      [](const Keyframe &_a, const Keyframe &_b)
      {
        return _a.time < _b.time;
      });

  this->dataPtr->loop = _sdf->Get<bool>("loop", false).first;
  if (this->dataPtr->loop)
  {
    const auto lastTime = keyframes.back().time;
    this->dataPtr->period = lastTime;
    if (_sdf->HasElement("period"))
    {
      const auto period = ToDuration(_sdf->Get<double>("period"));
      if (!period)
      {
        gzerr << "The [period] must be a non negative number of seconds. "
              << "Using the time of the last keyframe.\n";
      }
      else if (*period < lastTime)
      {
        gzwarn << "The last keyframe is later than the [period]. Extending "
               << "the period to "
               << std::chrono::duration<double>(lastTime).count() << " s.\n";
      }
      else
      {
        this->dataPtr->period = *period;
      }
    }

    if (this->dataPtr->period <= Duration::zero())
    {
      gzerr << "A looping timeline needs a [period] greater than zero. The "
            << "timeline will play once.\n";
      this->dataPtr->loop = false;
    }
  }

  const bool hasServices = std::any_of(keyframes.begin(), keyframes.end(),
      [](const Keyframe &_keyframe)
      {
        return !_keyframe.services.empty();
      });
  if (hasServices)
  {
    this->dataPtr->serviceThread = std::thread(
        &TimelinePublisherPrivate::ServiceWorker, this->dataPtr.get());
  }
}

//////////////////////////////////////////////////
void TimelinePublisher::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("TimelinePublisher::PreUpdate");
  // Keyframes at time zero wait for the simulation to start.
  if (_info.paused)
    return;

  this->dataPtr->Update(_info.simTime);
}

//////////////////////////////////////////////////
void TimelinePublisher::Reset(const UpdateInfo &/*_info*/,
    EntityComponentManager &/*_ecm*/)
{
  this->dataPtr->Restart();
}

//////////////////////////////////////////////////
void TimelinePublisherPrivate::Restart()
{
  this->cycleStart.reset();

  std::lock_guard<std::mutex> lock(this->serviceMutex);
  this->pendingServices.clear();
}

//////////////////////////////////////////////////
void TimelinePublisherPrivate::Update(const Duration &_simTime)
{
  if (this->keyframes.empty())
    return;

  // The simulation time went back without a reset, for example after a seek.
  if (this->cycleStart && _simTime < this->lastSimTime)
    this->Restart();
  this->lastSimTime = _simTime;

  if (!this->cycleStart)
  {
    this->cycleStart = _simTime;
    this->next = 0;
    this->cycle = 0;
  }

  while (true)
  {
    const auto elapsed = _simTime - *this->cycleStart;
    for (; this->next < this->keyframes.size() &&
           this->keyframes[this->next].time <= elapsed; ++this->next)
    {
      this->Fire(this->keyframes[this->next]);
    }

    // The period is at least the time of the last keyframe, so every
    // keyframe of the cycle has fired once the period is over.
    if (!this->loop || elapsed < this->period)
      return;

    // Skip the cycles that the step covers entirely, for example when the
    // period is shorter than the step or the simulation time jumps forward.
    // Only the cycle in progress fires.
    const auto cycles = elapsed / this->period;
    *this->cycleStart += cycles * this->period;
    this->next = 0;
    this->cycle += static_cast<std::size_t>(cycles);
    this->DropStaleServices();
  }
}

//////////////////////////////////////////////////
void TimelinePublisherPrivate::Fire(Keyframe &_keyframe)
{
  for (auto &output : _keyframe.outputs)
    output.pub.Publish(*output.msg);

  if (_keyframe.services.empty())
    return;

  {
    std::lock_guard<std::mutex> lock(this->serviceMutex);
    for (const auto &call : _keyframe.services)
      this->pendingServices.push_back({&call, this->cycle});
  }
  this->serviceCv.notify_one();
}

//////////////////////////////////////////////////
void TimelinePublisherPrivate::DropStaleServices()
{
  std::lock_guard<std::mutex> lock(this->serviceMutex);
  std::size_t dropped{0};
  for (; !this->pendingServices.empty() &&
         this->pendingServices.front().cycle + 1 < this->cycle; ++dropped)
  {
    this->pendingServices.pop_front();
  }

  if (dropped > 0 && !this->droppedWarned)
  {
    gzwarn << "Service calls are taking longer than the timeline period. "
           << "Dropping calls from earlier cycles.\n";
    this->droppedWarned = true;
  }
}

//////////////////////////////////////////////////
void TimelinePublisherPrivate::ServiceWorker()
{
  while (true)
  {
    const ServiceCall *call{nullptr};
    {
      std::unique_lock<std::mutex> lock(this->serviceMutex);
      this->serviceCv.wait(lock, [this]
          {
            return this->done || !this->pendingServices.empty();
          });
      if (this->done)
        return;
      call = this->pendingServices.front().call;
      this->pendingServices.pop_front();
    }

    std::unique_ptr<google::protobuf::Message> rep(call->rep->New());
    bool result{false};
    if (!this->node.Request(call->name, *call->req, call->timeout, *rep,
                            result))
    {
      gzerr << "Service call [" << call->name << "] timed out.\n";
    }
    else if (!result)
    {
      gzerr << "Service call [" << call->name << "] failed.\n";
    }
  }
}

GZ_ADD_PLUGIN(TimelinePublisher,
              System,
              TimelinePublisher::ISystemConfigure,
              TimelinePublisher::ISystemPreUpdate,
              TimelinePublisher::ISystemReset)

GZ_ADD_PLUGIN_ALIAS(TimelinePublisher,
                    "gz::sim::systems::TimelinePublisher")
