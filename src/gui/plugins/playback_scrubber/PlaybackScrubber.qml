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
  columns: 2
  rowSpacing: 0
  Layout.minimumWidth: 430
  Layout.minimumHeight: 170
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  /**
   * True if the playback scrubber is currently being pressed/dragged.
   */
  property var isPressed: false

  /**
   * The current time the playback scrubber is at.
   */
  property var currentTime: ""

  /**
   * The start time of the log playback file.
   */
  property var startTime: ""

  /**
   * The end time of the log playback file.
   */
  property var endTime: ""

  /**
   * Update the slider to the new values if it is currently being dragged.
   */
  function updateSliderValue() {
    if (!playbackScrubber.isPressed)
    {
      slider.value = PlaybackScrubber.Progress();
    }
  }

  /**
   * Update the end time of the log playback file.
   */
  function updateEndTime() {
    endTime = PlaybackScrubber.EndTimeAsString();
  }

  /**
   * Update the start time of the log playback file.
   */
  function updateStartTime() {
    startTime = PlaybackScrubber.CurrentTimeAsString();
  }

  /**
   * Update the current time the playback scrubber is at.
   */
  function updateCurrentTime() {
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
  Slider {
    id: slider
    height: 40
    Layout.fillWidth: true
    Layout.columnSpan: 2
    from: 0
    value: updateSliderValue()
    to: 1
    stepSize: 0.001
    topPadding: 17
    onPressedChanged: {
      if (!pressed)
      {
        PlaybackScrubber.OnDrop(slider.value);
        playbackScrubber.isPressed = false;
      }
      else
      {
        playbackScrubber.isPressed = true;
      }
    }
  }

  TextField {
    id: textField
    height: 40
    width: 30
    Layout.columnSpan: 1
    Layout.alignment: Qt.AlignRight
    placeholderText: currentTime
    onAccepted: {
      PlaybackScrubber.OnTimeEntered(textField.text);
      textField.text = "";
    }
    color: Material.theme == Material.Light ? "black" : "white"
  }

  Text {
    id: maxTime
    height: 40
    width: 30
    Layout.columnSpan: 1
    text: qsTr("/   ") + endTime
    Layout.alignment: Qt.AlignLeft
    font.pointSize: 11.5
    color: Material.theme == Material.Light ? "black" : "white"
  }

  // Bottom spacer
  Item {
    Layout.columnSpan: 2
    height: 10
    Layout.fillWidth: true
  }
}
