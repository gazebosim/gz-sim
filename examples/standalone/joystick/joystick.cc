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

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>
#include <gz/msgs/joy.pb.h>
#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: joystick <sdf_file>" << std::endl;
    return -1;
  }

  // Get parameters from SDF file
  auto sdf = sdf::readFile(argv[1]);

  if (!sdf)
  {
    std::cerr << "Failed to parse file [" << argv[1] << "]" << std::endl;
    return -1;
  }

  // TODO(louise) This is not a plugin, we need a new SDF tag or smth
  auto joy = sdf->Root()->GetElement("world")->GetElement("plugin");

  // Get the name of the joystick device.
  auto deviceFilename = joy->Get<std::string>("dev", "/dev/input/js0").first;

  bool opened = false;
  int joyFd = -1;

  // Attempt to open the joystick
  for (int i = 0; i < 10 && !opened; ++i)
  {
    joyFd = open(deviceFilename.c_str(), O_RDONLY);

    if (joyFd != -1)
    {
      // Close and open the device to get a better initial state.
      close(joyFd);
      joyFd = open(deviceFilename.c_str(), O_RDONLY);
      opened = true;
    }
    else
    {
      std::cout << "Unable to open joystick at [" << deviceFilename
        << "] Attemping again\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // Stop if we couldn't open the joystick after N attempts
  if (joyFd == -1)
  {
    std::cerr << "Unable to open joystick at [" << deviceFilename
          << "]. The joystick will not work.\n";
    return -1;
  }

  auto stickyButtons = joy->Get<bool>("sticky_buttons", false).first;

  // Read the amount of dead zone for the analog joystick
  float deadzone = gz::math::clamp(
      joy->Get<float>("dead_zone", 0.05f).first,
      0.0f, 0.9f);

  // Read the rate at which data should be published
  float interval = 1.0f;
  float intervalRate = joy->Get<float>("rate", 1.0f).first;
  if (intervalRate <= 0)
    interval = 1.0f;
  else
    interval = 1.0f / intervalRate;

  // Read the rate at which joystick data should be accumulated into
  // a message.
  float accumulationInterval = 0.001f;
  float accumulationRate = joy->Get<float>("accumulation_rate", 1000).first;
  if (accumulationRate <= 0)
    accumulationInterval = 0.0f;
  else
    accumulationInterval = 1.0f / accumulationRate;

  // Check that we are not publishing faster than accumulating data. This is
  // not a critical error, but doesn't make a whole lot of sense.
  if (interval < accumulationInterval)
  {
    std::cout << "The publication rate of [" << 1.0 / interval
      << " Hz] is greater than the accumulation rate of ["
      << 1.0 / accumulationInterval
      << " Hz]. Timing behavior is ill defined.\n";
  }

  auto unscaledDeadzone = 32767.0f * deadzone;
  auto axisScale = -1.0f / (1.0f - deadzone) / 32767.0f;

  auto topic = joy->Get<std::string>("topic", "/joy").first;

  // Create the publisher of joy messages
  gz::transport::Node node;
  auto pub = node.Advertise<gz::msgs::Joy>(topic);

  fd_set set;
  struct timeval tv;
  bool timeoutSet = false;
  bool accumulate = false;
  bool accumulating = false;

  gz::msgs::Joy joyMsg;
  gz::msgs::Joy lastJoyMsg;
  gz::msgs::Joy stickyButtonsJoyMsg;

  auto stop{false};
  while (!stop)
  {
    FD_ZERO(&set);
    FD_SET(joyFd, &set);

    int selectOut = select(joyFd+1, &set, NULL, NULL, &tv);

    if (selectOut == -1)
    {
      tv.tv_sec = 0;
      tv.tv_usec = 0;

      std::cout << "Joystick might be closed\n";
      if (!stop)
        continue;
      else
        break;
    }
    else if (stop)
      break;

    js_event event;

    if (FD_ISSET(joyFd, &set))
    {
      if (read(joyFd, &event, sizeof(js_event)) == -1 && errno != EAGAIN)
      {
        std::cout << "Joystick read failed, might be closed\n";
        return -1;
      }

      float value = event.value;
      switch (event.type)
      {
        case JS_EVENT_BUTTON:
        case JS_EVENT_BUTTON | JS_EVENT_INIT:
          {
            // Update number of buttons
            if (event.number >= joyMsg.buttons_size())
            {
              joyMsg.mutable_buttons()->Resize(event.number+1, 0.0f);
              lastJoyMsg.mutable_buttons()->Resize(event.number+1, 0.0f);
              stickyButtonsJoyMsg.mutable_buttons()->Resize(
                  event.number+1, 0.0f);
            }

            // Update the button
            joyMsg.set_buttons(event.number,
                !gz::math::equal(value, 0.0f) ? 1 : 0);

            // For initial events, wait a bit before sending to try to catch
            // all the initial events.
            accumulate = !(event.type & JS_EVENT_INIT);
            break;
          }
        case JS_EVENT_AXIS:
        case JS_EVENT_AXIS | JS_EVENT_INIT:
          {
            if (event.number >= joyMsg.axes_size())
            {
              joyMsg.mutable_axes()->Resize(event.number+1, 0.0f);
              lastJoyMsg.mutable_axes()->Resize(event.number+1, 0.0f);
              stickyButtonsJoyMsg.mutable_axes()->Resize(
                  event.number+1, 0.0f);
            }

            // Smooth the deadzone
            if (value < -unscaledDeadzone)
              value += unscaledDeadzone;
            else if (value > unscaledDeadzone)
              value -= unscaledDeadzone;
            else
              value = 0.0f;

            joyMsg.set_axes(event.number, value * axisScale);

            // Will wait a bit before sending to try to combine events.
            accumulate = true;
            break;
          }
        default:
          {
            std::cout << "Unknown event type: time[" << event.time << "] "
              << "value[" << value << "] "
              << "type[" << event.type << "h] "
              << "number["<< event.number << "]" << std::endl;
            break;
          }
       }
     }
     // Assume that the timer has expired.
     else if (timeoutSet)
       accumulate = false;

    if (!accumulate)
    {
      if (stickyButtons)
      {
        // process each button
        for (int i = 0; i < joyMsg.buttons_size(); ++i)
        {
          // change button state only on transition from 0 to 1
          if (joyMsg.buttons(i) == 1 && lastJoyMsg.buttons(i) == 0)
          {
            stickyButtonsJoyMsg.set_buttons(i,
              stickyButtonsJoyMsg.buttons(i) ? 0 : 1);
          }
        }

        // update last published message
        lastJoyMsg = joyMsg;

        // Copy the axis
        stickyButtonsJoyMsg.mutable_axes()->CopyFrom(joyMsg.axes());

        // Publish the stick buttons message
        pub.Publish(stickyButtonsJoyMsg);
      }
      else
      {
        // Publish the normal joy message
        pub.Publish(joyMsg);
      }

      timeoutSet = false;
      accumulating = false;
      accumulate = false;
    }

    // If an axis event occurred, start a timer to combine with other events.
    if (!accumulating && accumulate)
    {
      tv.tv_sec = trunc(accumulationInterval);
      tv.tv_usec = (accumulationInterval - tv.tv_sec) * 1e6;
      accumulating = true;
      timeoutSet = true;
    }

    // Set a timeout for the signal call at the beginning of this loop.
    if (!timeoutSet)
    {
      tv.tv_sec = trunc(interval);
      tv.tv_usec = (interval - tv.tv_sec) * 1e6;
      timeoutSet = true;
    }
   }

  // Close the joystick
  close(joyFd);
}
