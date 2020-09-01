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

GridLayout {
  id: playbackScrubber
  columns: 5
  Layout.minimumWidth: 100
  Layout.minimumHeight: 200
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10
  property var isPressed: false
  property var currentTime: ""

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
    currentTime = PlaybackScrubber.CurrentTimeAsString();
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

  /*
  // Top spacer
  Item {
    Layout.columnSpan: 6
    height: 12
    Layout.fillWidth: true
  }
  */ 
  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  // Top spacer
  Item {
    Layout.columnSpan: 3
    Layout.fillWidth: true
    height: 12
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  Text {
    text: "Current time: " + currentTime
    Layout.columnSpan: 3
    color: "dimgrey"
    font.bold: true
  }


  Text {
    id: startTime
    font.pointSize: 9
    Layout.columnSpan: 1
    text: PlaybackScrubber.StartTimeAsString()
    color: "dimgrey"
  }
  Slider {
    id: slider
    from: 0
    value: updateSliderValue()
    to: 1
    stepSize: 0.001
    Layout.columnSpan: 1
    Layout.fillWidth: true
    Layout.alignment: Qt.AlignVCenter
    Layout.leftMargin: 0
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
    Layout.columnSpan: 1
    text: PlaybackScrubber.EndTimeAsString()
    color: "dimgrey"
  }
  
  TextField {
    id: textField
    Layout.columnSpan: 1
    placeholderText: PlaybackScrubber.CurrentTimeAsString()
    onAccepted: {
      PlaybackScrubber.OnTimeEntered(textField.text);
    }
  }
  Text {
    id: maxTime
    text: qsTr("/ ") + PlaybackScrubber.EndTimeAsString()
    Layout.columnSpan: 2
  }

  // Bottom spacer
  Item {
    Layout.columnSpan: 3
    height: 12
    Layout.fillWidth: true
  }
}
