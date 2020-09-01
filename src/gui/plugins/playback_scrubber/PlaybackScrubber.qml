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
  Layout.minimumWidth: 425
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
    width: 30
  }

  Text {
    text: "Current time: " + currentTime
    Layout.columnSpan: 3
    color: Material.theme == Material.Light ? "black" : "white"
    font.bold: true
  }

  Row {
    Layout.columnSpan: 3
    spacing: 3
    Rectangle {
      height: 50
      width: 80
      color: "transparent"
      Text {
        id: startTime
        font.pointSize: 9
        text: PlaybackScrubber.StartTimeAsString()
        color: Material.theme == Material.Light ? "black" : "white"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        
      }
    }
    Rectangle {
      height: 50
      width: 220
      color: "transparent"
      Slider {
        id: slider
        from: 0
        value: updateSliderValue()
        to: 1
        stepSize: 0.001
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
    }
    Rectangle {
      height: 50
      width: 80
      color: "transparent"
      Text {
        id: endTime
        font.pointSize: 9
        text: PlaybackScrubber.EndTimeAsString()
        color: Material.theme == Material.Light ? "black" : "white"
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
      }
    }
  }

  TextField {
    id: textField
    Layout.columnSpan: 1
    placeholderText: PlaybackScrubber.CurrentTimeAsString()
    onAccepted: {
      PlaybackScrubber.OnTimeEntered(textField.text);
    }
    color: Material.theme == Material.Light ? "black" : "white"
  }
  Text {
    id: maxTime
    text: qsTr("/ ") + PlaybackScrubber.EndTimeAsString()
    Layout.columnSpan: 1
    Layout.alignment: Qt.AlignLeft
    color: Material.theme == Material.Light ? "black" : "white"
  }
  Item {
    Layout.columnSpan: 1
    Layout.fillWidth: true
  }

  // Bottom spacer
  Item {
    Layout.columnSpan: 3
    height: 12
    Layout.fillWidth: true
  }
}
