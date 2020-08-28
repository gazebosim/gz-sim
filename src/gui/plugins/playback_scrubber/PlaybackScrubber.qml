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
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/qml"

ToolBar {
  id: playbackScrubber
  Layout.minimumWidth: 200
  Layout.minimumHeight: 300
  property var isPressed: false

  function updateSliderValue() {
    if (!playbackScrubber.isPressed)
    {
      slider.value = PlaybackScrubber.Progress();
    }
  }

  function updateEndTime() {
    endTime.text = PlaybackScrubber.EndTimeAsString();
    maxTime.text = qsTr("/ ") + PlaybackScrubber.EndTimeAsString();
  }

  function updateStartTime() {
    startTime.text = PlaybackScrubber.StartTimeAsString();
  }

  function updateCurrentTime() {
    textField.placeholderText = PlaybackScrubber.CurrentTimeAsString();
    currentTime.text = PlaybackScrubber.CurrentTimeAsString();
  }

  background: Rectangle {
    color: "transparent"
  }

  Connections {
    target: PlaybackScrubber
    onNewProgress: {
      updateSliderValue();
      updateStartTime();
      updateEndTime();
      updateCurrentTime();
    }
  }

  Column {
    anchors.fill: parent
    spacing: 2
    topPadding: 10
    leftPadding: 10
    RowLayout {
      Text {
        text: "Current time: "
      }
      Text {
        id: currentTime
        text: PlaybackScrubber.CurrentTimeAsString()
      }
    }
    RowLayout {
      Text {
        id: startTime
        font.pointSize: 9
        text: PlaybackScrubber.StartTimeAsString()
      }
      Slider {
        id: slider
        from: 0
        value: updateSliderValue()
        to: 1
        stepSize: 0.001
        /*
         onValueChanged: {
           print(slider.value)
         }
         */
        onPressedChanged: {
          if (!pressed)
          {
            PlaybackScrubber.OnDrag(slider.value);
            playbackScrubber.isPressed = false;
          }
          else
          {
            playbackScrubber.isPressed = true;
          }
        }
      }
      Text {
        id: endTime
        font.pointSize: 9
        text: PlaybackScrubber.EndTimeAsString()
      }
    }
    RowLayout {
      TextField {
        id: textField
        placeholderText: PlaybackScrubber.CurrentTimeAsString()
        onAccepted: {
          PlaybackScrubber.OnTimeEntered(textField.text);
        }
      }
      Text {
        id: maxTime
        text: qsTr("/ ") + PlaybackScrubber.EndTimeAsString()
      }
    }
  }
}
