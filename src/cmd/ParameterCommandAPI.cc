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

#include "ParameterCommandAPI.hh"

#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/parameter.pb.h>
#include <ignition/msgs/parameter_declarations.pb.h>
#include <ignition/msgs/parameter_name.pb.h>
#include <ignition/msgs/parameter_value.pb.h>

#include <ignition/transport/Node.hh>

#include "Utils.hh"

using namespace ignition;

//////////////////////////////////////////////////
extern "C" void cmdParametersList()
{
  std::string worldName = gazebo::cmd::getWorldName();
  transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world/" + worldName + "/list_parameters"};

  msgs::Empty req;
  msgs::ParameterDeclarations res;

  std::cout << std::endl << "Listing parameters for world [" << worldName
            << "]..." << std::endl << std::endl;

  if (!node.Request(service, req, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service << "] failed unexpectedly"
              << std::endl;
    return;
  }

  if (!res.parameter_declarations_size()) {
    std::cout << "No parameters available" << std::endl;
    return;
  }
  for (const auto & decl : res.parameter_declarations()) {
    std::cout << decl.name() << "            [" << decl.type() << "]"
              << std::endl;
  }
}

//////////////////////////////////////////////////
extern "C" void cmdParameterGet(const char *_paramName) {
  std::string worldName = gazebo::cmd::getWorldName();
  transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world/" + worldName + "/get_parameter"};

  msgs::ParameterName req;
  msgs::ParameterValue res;

  req.set_name(_paramName);

  std::cout << std::endl << "Getting parameter [" << _paramName
            << "] for world [" << worldName << "]..." << std::endl
            << std::endl;

  if (!node.Request(service, req, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service
              << "] failed." << std::endl <<
              "There may be not parameter named like that." << std::endl;
    return;
  }
  const auto & msgType = res.type();
  std::cout << "Parameter type [" << msgType << "" << std::endl
            << "------------------------------------------------"
            << std::endl;
  auto msg = ignition::msgs::Factory::New(msgType);
  if (!msg) {
    std::cerr << "Could not create a message of type [" << msgType
              << "]." << std::endl << "The message type may be invalid."
              << std::endl;
    return;
  }
  std::istringstream istr(res.value());
  msg->ParseFromIstream(&istr);
  google::protobuf::io::OstreamOutputStream fos{&std::cout};
  if (!google::protobuf::TextFormat::Print(*msg, &fos)) {
    std::cerr << "failed to convert the parameter value to a string"
              << std::endl;
    return;
  }
  std::cout << std::endl
            << "------------------------------------------------"
            << std::endl;
}

extern "C" void cmdParameterSet(
    const char *_paramName, const char * _paramType, const char *_paramValue)
{
  std::string worldName = gazebo::cmd::getWorldName();
  transport::Node node;

  bool result{false};
  const unsigned int timeout{5000};
  const std::string service{"/world" + worldName + "/set_parameter"};

  msgs::Parameter req;
  msgs::Empty res;

  auto msg = ignition::msgs::Factory::New(_paramType, _paramValue);
  if (!msg) {
    // try again, to check if the type name was valid
    auto defaultMsg = ignition::msgs::Factory::New(_paramType);
    std::cerr << "Could not create a message of type [" << _paramType << "]."
              << std::endl;
    if (!defaultMsg) {
      std::cerr << "The message type may be invalid." << std::endl;
      return;
    }
    std::cerr << "The message string representation may be invalid." << std::endl;
    return;
  }
  std::ostringstream oss;
  msg->SerializeToOstream(&oss);
  req.set_name(_paramName);
  req.set_type(_paramType);
  req.set_value(oss.str());

  std::cout << std::endl << "Setting parameter [" << _paramName
            << "] for world [" << worldName << "]..." << std::endl
            << std::endl;

  if (!node.Request(service, req, timeout, res, result))
  {
    std::cerr << std::endl << "Service call to [" << service << "] timed out"
              << std::endl;
    return;
  }

  if (!result)
  {
    std::cerr << std::endl << "Service call to [" << service
              << "] failed. There may be not parameter named like that."
              << std::endl;
    return;
  }
  std::cout << "Parameter successfully set!" << std::endl;
}
