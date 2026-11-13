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

// A bar split into parts. Each part is {label, value, color}, and its value is
// a fraction of 1. `whole` is the value that fills the bar, and what the parts
// leave of it shows as the empty track.
Item {
  id: bar

  property var parts: []
  property real whole: 1.0
  property color trackColor: "transparent"
  property color textColor: "white"
  property bool showValues: false

  readonly property var shown: parts.filter(function(part) {
    return part.value > 0
  })
  readonly property real sum: shown.reduce(function(total, part) {
    return total + part.value
  }, 0)
  readonly property real fullScale: Math.max(whole, sum)
  readonly property int gap: 2
  readonly property real usable:
    Math.max(0, width - gap * Math.max(0, shown.length - 1))
  readonly property real cornerRadius: Math.min(4, height / 2)

  implicitHeight: 24

  Rectangle {
    anchors.fill: parent
    radius: bar.cornerRadius
    color: bar.trackColor
  }

  Row {
    spacing: bar.gap

    Repeater {
      model: bar.shown

      Rectangle {
        id: part

        required property var modelData
        readonly property string percent:
          Math.round(100 * modelData.value) + "%"

        width: bar.fullScale > 0 ? bar.usable * modelData.value / bar.fullScale : 0
        height: bar.height
        radius: bar.cornerRadius
        color: modelData.color

        Label {
          anchors.verticalCenter: parent.verticalCenter
          x: 6
          visible: bar.showValues && part.width >= implicitWidth + 12
          text: part.percent
          color: bar.textColor
          font.pixelSize: 11
          font.bold: true
        }

        MouseArea {
          id: hover
          anchors.fill: parent
          hoverEnabled: true
          acceptedButtons: Qt.NoButton
          ToolTip.visible: hover.containsMouse
          ToolTip.delay: 300
          ToolTip.text: part.modelData.label + ": " + part.percent
        }
      }
    }
  }
}
