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
import QtQuick 2.15
import QtQuick.Controls 2.15

// One column per report: its height is the real time factor and its color says
// what took most of the time. The newest report is on the right.
Item {
  id: history

  // Each sample is {rtf, limited, location}
  property var samples: []
  property real target: 0
  property int capacity: 40
  property color targetColor: "gray"
  // Returns the color of a sample
  property var colorOf: function(sample) { return "gray" }

  readonly property real ceiling: {
    var highest = target * 1.1
    for (var i = 0; i < samples.length; ++i)
      highest = Math.max(highest, samples[i].rtf * 1.05)
    return Math.max(highest, 0.01)
  }
  readonly property int spacing: 1
  readonly property real columnWidth:
    Math.max(1, (width - spacing * (capacity - 1)) / capacity)

  implicitHeight: 36

  Row {
    anchors.right: parent.right
    height: parent.height
    spacing: history.spacing

    Repeater {
      model: history.samples

      Rectangle {
        required property var modelData

        anchors.bottom: parent.bottom
        width: history.columnWidth
        height: Math.max(1, modelData.rtf / history.ceiling * history.height)
        color: history.colorOf(modelData)
      }
    }
  }

  Rectangle {
    visible: history.target > 0
    y: history.height * (1 - history.target / history.ceiling)
    width: parent.width
    height: 1
    color: history.targetColor
  }

  MouseArea {
    id: hover

    readonly property int sampleIndex: {
      var fromEnd = Math.floor((history.width - mouseX) /
          (history.columnWidth + history.spacing))
      return history.samples.length - 1 - fromEnd
    }
    readonly property var sample: history.samples[sampleIndex]

    anchors.fill: parent
    hoverEnabled: true
    acceptedButtons: Qt.NoButton
    ToolTip.visible: hover.containsMouse && hover.sample !== undefined
    ToolTip.delay: 100
    ToolTip.text: hover.sample === undefined ? "" :
      "RTF " + hover.sample.rtf.toFixed(2) + (hover.sample.limited ?
          ", limited in " + hover.sample.location : ", keeping up")
  }
}
