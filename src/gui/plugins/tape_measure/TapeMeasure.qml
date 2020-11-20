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
  id: tapeMeasure
  Layout.minimumWidth: 250
  Layout.minimumHeight: 100

  property var distance: 0.0

  function updateDistance() {
    distance = TapeMeasure.Distance();
  }

  function startMeasurement() {
    TapeMeasure.OnReset();
    TapeMeasure.OnMeasure();
  }

  Connections {
    target: TapeMeasure
    onNewDistance: {
      updateDistance();
    }
  }

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  Shortcut {
    sequence: "M"
    onActivated: startMeasurement()
  }

  RowLayout {
    spacing: 1
    ToolButton {
      id: select
      checkable: true
      checked: true
      ButtonGroup.group: group
      ToolTip.text: "Tape Measure Tool"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "tape_measure.png"
        sourceSize.width: 36;
        sourceSize.height: 36;
      }
      onClicked: {
        TapeMeasure.OnMeasure();
      }
    }
    ToolButton {
      id: reset
      checkable: true
      checked: true
      ButtonGroup.group: group
      ToolTip.text: "Reset measurement"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "trashcan.png"
        sourceSize.width: 36;
        sourceSize.height: 36;
      }
      onClicked: {
        TapeMeasure.OnReset();
      }
    }
    Text {
      text: qsTr(" Distance (m): " + distance.toFixed(3))
    }
  }
}
