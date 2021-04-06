/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "TriggeredPublisher.hh"

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/message_differencer.h>

#include <limits>
#include <utility>

#include <ignition/common/Profiler.hh>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>

// bug https://github.com/protocolbuffers/protobuf/issues/5051
#ifdef _WIN32
#undef GetMessage
#endif

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Base class for input matchers.
class systems::InputMatcher
{
  /// \brief Constructor
  /// \param[in] _msgType Input message type
  public: InputMatcher(const std::string &_msgType);

  /// \brief Destructor
  public: virtual ~InputMatcher() = default;

  /// \brief Match input message against the match criteria.
  /// \param[in] _input Input message
  /// \return True if the input matches the match criteria.
  public: bool Match(const transport::ProtoMsg &_input) const;

  /// \brief Match input message against the match criteria. Subclasses should
  /// override this
  /// \param[in] _input Input message
  /// \return True if the input matches the match criteria.
  public: virtual bool DoMatch(const transport::ProtoMsg &_input) const = 0;

  /// \brief Checks if the matcher is in a valid state.
  /// \return True if the matcher is in a valid state.
  public: virtual bool IsValid() const;

  /// \brief Set the float comparison tolerance
  /// \param[in] _tol Tolerance for float comparisons
  public: void SetTolerance(double _tol);

  /// \brief Helper function that checks if two messages have the same type
  /// \input[in] _matcher Matcher message
  /// \input[in] _input Input message
  /// \return True if the two message types match
  public: static bool CheckTypeMatch(const transport::ProtoMsg &_matcher,
                                     const transport::ProtoMsg &_input);

  /// \brief Factory function for creating matchers.
  /// \param[in] _msgType Input message type (eg. ignition.msgs.Boolean)
  /// \param[in] _matchElem the SDFormat Element that contains the configuration
  /// for the matcher
  /// \return A concrete InputMatcher initialized according to the contents of
  /// _matchElem. A nullptr is returned if the created InputMatcher is invalid.
  public: static std::unique_ptr<InputMatcher> Create(
              const std::string &_msgType, const sdf::ElementPtr &_matchElem);

  /// \brief Protobuf message for matching against input
  protected: std::unique_ptr<transport::ProtoMsg> matchMsg;

  /// \brief State of the matcher
  protected: bool valid{false};

  /// \brief Field comparator used by MessageDifferencer. This is where
  /// tolerance for float comparisons is set
  protected: google::protobuf::util::DefaultFieldComparator comparator;

  /// \brief MessageDifferencer used for comparing input to matcher. This is
  /// mutable because MessageDifferencer::CompareWithFields is not a const
  /// function
  protected: mutable google::protobuf::util::MessageDifferencer diff;
};

//////////////////////////////////////////////////
/// \brief Matches any input message of the specified type
class AnyMatcher : public InputMatcher
{
  /// \brief Constructor
  /// \param[in] _msgType Input message type
  public: explicit AnyMatcher(const std::string &_msgType);

  // Documentation inherited
  public: bool DoMatch(const transport::ProtoMsg &_input) const override;
};


//////////////////////////////////////////////////
/// \brief Matches the whole input message against the match criteria. Floats
/// are compared using MathUtil::AlmostEquals()
class FullMatcher : public InputMatcher
{
  /// \brief Constructor
  /// \param[in] _msgType Input message type
  /// \param[in] _logicType Determines what the returned value of Match() would
  /// be on a successful comparison. If this is false, a successful match would
  /// return false.
  /// \param[in] _matchString String used to construct the protobuf message
  /// against which input messages are matched. This is the human-readable
  /// representation of a protobuf message as used by `ign topic` for publishing
  /// messages
  public: FullMatcher(const std::string &_msgType, bool _logicType,
                      const std::string &_matchString);

  // Documentation inherited
  public: bool DoMatch(const transport::ProtoMsg &_input) const override;

  /// \brief Logic type of this matcher
  protected: const bool logicType;
};

//////////////////////////////////////////////////
/// \brief Matches a specific field in the input message against the match
/// criteria. Floats are compared using MathUtil::AlmostEquals()
class FieldMatcher : public InputMatcher
{
  /// \brief Constructor
  /// \param[in] _msgType Input message type
  /// \param[in] _logicType Determines what the returned value of Match() would
  /// be on a successful comparison. If this is false, a successful match would
  /// return false.
  /// \param[in] _fieldName Name of the field to compare
  /// \param[in] _fieldString String used to construct the protobuf message
  /// against which the specified field in the input messages are matched. This
  /// is the human-readable representation of a protobuf message as used by `ign
  /// topic` for publishing messages
  public: FieldMatcher(const std::string &_msgType, bool _logicType,
                       const std::string &_fieldName,
                       const std::string &_fieldString);

  // Documentation inherited
  public: bool DoMatch(const transport::ProtoMsg &_input) const override;

  /// \brief Helper function to find a subfield inside the message based on the
  /// given field name.
  /// \param[in] _msg The message containing the subfield
  /// \param[in] _fieldName Field name inside the message. Each period ('.')
  /// character is used to indicate a subfield.
  /// \param[out] _fieldDesc Field descriptors found while traversing the
  /// message to find the field
  /// \param[out] _subMsg Submessage of the field that corresponds to the field
  /// name
  protected: static bool FindFieldSubMessage(
                 transport::ProtoMsg *_msg, const std::string &_fieldName,
                 std::vector<const google::protobuf::FieldDescriptor *>
                     &_fieldDesc,
                 transport::ProtoMsg **_subMsg);

  /// \brief Logic type of this matcher
  protected: const bool logicType;

  /// \brief Name of the field compared by this matcher
  protected: const std::string fieldName;

  /// \brief Field descriptor of the field compared by this matcher
  protected: std::vector<const google::protobuf::FieldDescriptor *>
                 fieldDescMatcher;
};

//////////////////////////////////////////////////
InputMatcher::InputMatcher(const std::string &_msgType)
    : matchMsg(msgs::Factory::New(_msgType))
{
  this->comparator.set_float_comparison(
      google::protobuf::util::DefaultFieldComparator::APPROXIMATE);

  this->diff.set_field_comparator(&this->comparator);
}

//////////////////////////////////////////////////
bool InputMatcher::Match(const transport::ProtoMsg &_input) const
{
  if (!this->CheckTypeMatch(*this->matchMsg, _input))
  {
    return false;
  }
  return this->DoMatch(_input);
}

void InputMatcher::SetTolerance(double _tol)
{
  this->comparator.SetDefaultFractionAndMargin(
      std::numeric_limits<double>::min(), _tol);
}

//////////////////////////////////////////////////
bool InputMatcher::CheckTypeMatch(const transport::ProtoMsg &_matcher,
                                  const transport::ProtoMsg &_input)
{
  const auto *matcherDesc = _matcher.GetDescriptor();
  const auto *inputDesc = _input.GetDescriptor();
  if (matcherDesc != inputDesc)
  {
    ignerr << "Received message has a different type than configured in "
           << "<input>. Expected [" << matcherDesc->full_name() << "] got ["
           << inputDesc->full_name() << "]\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
AnyMatcher::AnyMatcher(const std::string &_msgType) : InputMatcher(_msgType)
{
  this->valid = (nullptr == this->matchMsg || !this->matchMsg->IsInitialized());
}

//////////////////////////////////////////////////
bool AnyMatcher::DoMatch(const transport::ProtoMsg &) const
{
  return true;
}

//////////////////////////////////////////////////
FullMatcher::FullMatcher(const std::string &_msgType, bool _logicType,
                         const std::string &_matchString)
    : InputMatcher(_msgType), logicType(_logicType)
{
  if (nullptr == this->matchMsg || !this->matchMsg->IsInitialized())
    return;

  this->valid = google::protobuf::TextFormat::ParseFromString(
      _matchString, this->matchMsg.get());
}

//////////////////////////////////////////////////
bool FullMatcher::DoMatch(const transport::ProtoMsg &_input) const
{
  return this->logicType == this->diff.Compare(*this->matchMsg, _input);
}

//////////////////////////////////////////////////
FieldMatcher::FieldMatcher(const std::string &_msgType, bool _logicType,
                           const std::string &_fieldName,
                           const std::string &_fieldString)
    : InputMatcher(_msgType),
      logicType(_logicType),
      fieldName(_fieldName)
{
  if (nullptr == this->matchMsg || !this->matchMsg->IsInitialized())
    return;

  transport::ProtoMsg *matcherSubMsg{nullptr};
  if (!FindFieldSubMessage(this->matchMsg.get(), _fieldName,
                          this->fieldDescMatcher, &matcherSubMsg))
  {
    return;
  }

  if (this->fieldDescMatcher.empty())
  {
    return;
  }
  else if (this->fieldDescMatcher.back()->is_repeated())
  {
    this->diff.set_scope(google::protobuf::util::MessageDifferencer::PARTIAL);
    this->diff.set_repeated_field_comparison(
        google::protobuf::util::MessageDifferencer::AS_SET);
  }

  if (nullptr == matcherSubMsg)
    return;

  bool result = google::protobuf::TextFormat::ParseFieldValueFromString(
      _fieldString, this->fieldDescMatcher.back(), matcherSubMsg);
  if (!result)
  {
    ignerr << "Failed to parse matcher string [" << _fieldString
           << "] for field [" << this->fieldName << "] of input message type ["
           << _msgType << "]\n";
    return;
  }

  this->valid = true;
}

//////////////////////////////////////////////////
bool FieldMatcher::FindFieldSubMessage(
    transport::ProtoMsg *_msg, const std::string &_fieldName,
    std::vector<const google::protobuf::FieldDescriptor *> &_fieldDesc,
    transport::ProtoMsg **_subMsg)
{
  const google::protobuf::Descriptor *fieldMsgType = _msg->GetDescriptor();

  // If fieldMsgType is nullptr, then this is not a composite message and we
  // shouldn't be using a FieldMatcher
  if (nullptr == fieldMsgType)
  {
    ignerr << "FieldMatcher with field name [" << _fieldName
           << "] cannot be used because the input message type ["
           << fieldMsgType->full_name() << "] does not have any fields\n";
    return false;
  }

  *_subMsg = _msg;

  auto fieldNames = common::split(_fieldName, ".");
  if (fieldNames.empty())
  {
    ignerr << "Empty field attribute for input message type ["
           << fieldMsgType->full_name() << "]\n";
    return false;
  }

  for (std::size_t i = 0; i < fieldNames.size(); ++i)
  {
    auto fieldDesc = fieldMsgType->FindFieldByName(fieldNames[i]);

    if (nullptr == fieldDesc)
    {
      ignerr << "Field name [" << fieldNames[i]
             << "] could not be found in message type ["
             << fieldMsgType->full_name() << "].\n";
      return false;
    }

    _fieldDesc.push_back(fieldDesc);

    if (i < fieldNames.size() - 1)
    {
      if (google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE !=
          fieldDesc->cpp_type())
      {
        ignerr << "Subfield [" << fieldNames[i+1]
          << "] could not be found in Submessage type ["
          << fieldDesc->full_name() << "].\n";
        return false;
      }

      auto *reflection = (*_subMsg)->GetReflection();
      if (fieldDesc->is_repeated())
      {
        ignerr
            << "Field matcher for field name [" << _fieldName
            << "] could not be created because the field [" << fieldDesc->name()
            << "] is a repeated message type. Matching subfields of repeated "
            << "messages is not supported.\n";
        return false;
      }
      else
      {
        *_subMsg = reflection->MutableMessage(*_subMsg, fieldDesc);
      }

      // Update fieldMsgType for next iteration
      fieldMsgType = fieldDesc->message_type();
    }
  }

  return true;
}

//////////////////////////////////////////////////
bool FieldMatcher::DoMatch(
    const transport::ProtoMsg &_input) const
{
  google::protobuf::util::DefaultFieldComparator comp;

  auto *matcherRefl = this->matchMsg->GetReflection();
  auto *inputRefl = _input.GetReflection();
  const transport::ProtoMsg *subMsgMatcher = this->matchMsg.get();
  const transport::ProtoMsg *subMsgInput = &_input;
  for (std::size_t i = 0; i < this->fieldDescMatcher.size() - 1; ++i)
  {
    auto *fieldDesc = this->fieldDescMatcher[i];
    if (fieldDesc->is_repeated())
    {
      // This should not happen since the matching subfields of repeated fields
      // is not allowed and this matcher shouldn't have been created.
      ignerr << "Matching subfields of repeated messages is not supported\n";
    }
    else
    {
      subMsgMatcher = &matcherRefl->GetMessage(*subMsgMatcher, fieldDesc);
      subMsgInput = &inputRefl->GetMessage(*subMsgInput, fieldDesc);
    }
  }

  return this->logicType ==
         this->diff.CompareWithFields(*subMsgMatcher, *subMsgInput,
                                      {this->fieldDescMatcher.back()},
                                      {this->fieldDescMatcher.back()});
}

//////////////////////////////////////////////////
bool InputMatcher::IsValid() const
{
  return this->valid;
}

//////////////////////////////////////////////////
std::unique_ptr<InputMatcher> InputMatcher::Create(
    const std::string &_msgType, const sdf::ElementPtr &_matchElem)
{
  if (nullptr == _matchElem)
  {
    return std::make_unique<AnyMatcher>(_msgType);
  }

  std::unique_ptr<InputMatcher> matcher{nullptr};

  const auto logicTypeStr =
      _matchElem->Get<std::string>("logic_type", "positive").first;
  if (logicTypeStr != "positive" && logicTypeStr != "negative")
  {
    ignerr << "Unrecognized logic_type attribute [" << logicTypeStr
           << "] in matcher for input message type [" << _msgType << "]\n";
    return nullptr;
  }

  const bool logicType = logicTypeStr == "positive";

  auto inputMatchString = common::trimmed(_matchElem->Get<std::string>());
  if (!inputMatchString.empty())
  {
    if (_matchElem->HasAttribute("field"))
    {
      const auto fieldName = _matchElem->Get<std::string>("field");
      matcher = std::make_unique<FieldMatcher>(_msgType, logicType, fieldName,
                                               inputMatchString);
    }
    else
    {
      matcher =
          std::make_unique<FullMatcher>(_msgType, logicType, inputMatchString);
    }
    if (matcher == nullptr || !matcher->IsValid())
    {
      ignerr << "Matcher for input type [" << _msgType
             << "] could not be created from:\n"
             << inputMatchString << std::endl;
      return nullptr;
    }

    const auto tol = _matchElem->Get<double>("tol", 1e-8).first;
    matcher->SetTolerance(tol);
  }
  return matcher;
}

//////////////////////////////////////////////////
TriggeredPublisher::~TriggeredPublisher()
{
  this->done = true;
  this->newMatchSignal.notify_one();
  if (this->workerThread.joinable())
  {
    this->workerThread.join();
  }
}

//////////////////////////////////////////////////
void TriggeredPublisher::Configure(const Entity &,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &,
    EventManager &)
{
  sdf::ElementPtr sdfClone = _sdf->Clone();
  if (sdfClone->HasElement("input"))
  {
    auto inputElem = sdfClone->GetElement("input");
    this->inputMsgType = inputElem->Get<std::string>("type");
    if (this->inputMsgType.empty())
    {
      ignerr << "Input message type cannot be empty\n";
      return;
    }

    auto inTopic = inputElem->Get<std::string>("topic");
    this->inputTopic = transport::TopicUtils::AsValidTopic(inTopic);
    if (this->inputTopic.empty())
    {
      ignerr << "Invalid input topic [" << inTopic << "]" << std::endl;
      return;
    }

    if (inputElem->HasElement("match"))
    {
      for (auto matchElem = inputElem->GetElement("match"); matchElem;
           matchElem = matchElem->GetNextElement("match"))
      {
        auto matcher = InputMatcher::Create(this->inputMsgType, matchElem);
        if (nullptr != matcher)
        {
          this->matchers.push_back(std::move(matcher));
        }
      }
    }
    else
    {
      auto matcher = InputMatcher::Create(this->inputMsgType, nullptr);
      if (nullptr != matcher)
      {
        this->matchers.push_back(std::move(matcher));
      }
    }
  }
  else
  {
    ignerr << "No input specified" << std::endl;
    return;
  }

  if (this->matchers.empty())
  {
    ignerr << "No valid matchers specified\n";
    return;
  }

  if (sdfClone->HasElement("output"))
  {
    for (auto outputElem = sdfClone->GetElement("output"); outputElem;
         outputElem = outputElem->GetNextElement("output"))
    {
      OutputInfo info;
      info.msgType = outputElem->Get<std::string>("type");
      if (info.msgType.empty())
      {
        ignerr << "Output message type cannot be empty\n";
        continue;
      }
      auto topic = outputElem->Get<std::string>("topic");
      info.topic = transport::TopicUtils::AsValidTopic(topic);
      if (info.topic.empty())
      {
        ignerr << "Invalid topic [" << topic << "]" << std::endl;
        continue;
      }
      const std::string msgStr = outputElem->Get<std::string>();
      info.msgData = msgs::Factory::New(info.msgType, msgStr);
      if (nullptr != info.msgData)
      {
        info.pub =
            this->node.Advertise(info.topic, info.msgData->GetTypeName());
        if (info.pub.Valid())
        {
          this->outputInfo.push_back(std::move(info));
        }
        else
        {
          ignerr << "Output publisher could not be created for topic ["
                 << info.topic << "] with message type [" << info.msgType
                 << "]\n";
        }
      }
      else
      {
        ignerr << "Unable to create message of type [" << info.msgType
               << "] with data [" << msgStr << "] when creating output"
               << " publisher on topic " << info.topic << ".\n";
      }
    }
  }
  else
  {
    ignerr << "No ouptut specified" << std::endl;
    return;
  }

  auto msgCb = std::function<void(const transport::ProtoMsg &)>(
      [this](const auto &_msg)
      {
        if (this->MatchInput(_msg))
        {
          {
            std::lock_guard<std::mutex> lock(this->publishCountMutex);
            ++this->publishCount;
          }
          this->newMatchSignal.notify_one();
        }
      });
  if (!this->node.Subscribe(this->inputTopic, msgCb))
  {
    ignerr << "Input subscriber could not be created for topic ["
           << this->inputTopic << "] with message type [" << this->inputMsgType
           << "]\n";
    return;
  }

  std::stringstream ss;
  ss << "TriggeredPublisher subscribed on " << this->inputTopic
     << " and publishing on ";

  for (const auto &info : this->outputInfo)
  {
    ss << info.topic << ", ";
  }
  igndbg << ss.str() << "\n";

  this->workerThread =
      std::thread(std::bind(&TriggeredPublisher::DoWork, this));
}

//////////////////////////////////////////////////
void TriggeredPublisher::DoWork()
{
  while (!this->done)
  {
    std::size_t pending{0};
    {
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lock(this->publishCountMutex);
      this->newMatchSignal.wait_for(lock, 1s,
        [this]
        {
          return (this->publishCount > 0) || this->done;
        });

      if (this->publishCount == 0 || this->done)
        continue;

      std::swap(pending, this->publishCount);
    }

    for (auto &info : this->outputInfo)
    {
      for (std::size_t i = 0; i < pending; ++i)
      {
        info.pub.Publish(*info.msgData);
      }
    }
  }
}

//////////////////////////////////////////////////
bool TriggeredPublisher::MatchInput(const transport::ProtoMsg &_inputMsg)
{
  return std::all_of(this->matchers.begin(), this->matchers.end(),
                     [&](const auto &_matcher)
                     {
                       try
                       {
                         return _matcher->Match(_inputMsg);
                       } catch (const google::protobuf::FatalException &err)
                       {
                          ignerr << err.what() << std::endl;
                          return false;
                       }
                     });
}

IGNITION_ADD_PLUGIN(TriggeredPublisher,
                    ignition::gazebo::System,
                    TriggeredPublisher::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(TriggeredPublisher,
                          "ignition::gazebo::systems::TriggeredPublisher")
