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
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 350

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  GridLayout {
    columns: 4
    x: 35
    y: 10
    Text {
      text: "X:"
      font.weight: Font.Bold
    }
    ToolButton {
      id: x_min
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the top"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "x_min.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, -1)
      }
    }
    ToolButton {
      id: x_center
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Reset View Angle"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "x_center.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 0)
      }
    }
    ToolButton {
      id: x_max
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the left"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "x_max.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 1, 0)
      }
    }
    Text {
      text: "Y:"
      font.weight: Font.Bold
    }
    ToolButton {
      id: y_min
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the front"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "y_min.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(-1, 0, 0)
      }
    }
    ToolButton {
      id: y_center
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the right"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "y_center.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, -1, 0)
      }
    }
    ToolButton {
      id: y_max
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the back"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "y_max.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(1, 0, 0)
      }
    }
    Text {
      text: "Z:"
      font.weight: Font.Bold
    }
    ToolButton {
      id: z_min
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the bottom"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "z_min.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 1)
      }
    }
    ToolButton {
      id: z_center
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the bottom"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "z_center.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 1)
      }
    }
    ToolButton {
      id: z_max
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the bottom"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "z_max.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 1)
      }
    }
    CheckBox {
      text: qsTr("Reverse")
      Layout.columnSpan: 4
      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
    }
    ComboBox {
      width: 400
      Layout.columnSpan: 4
      model: ["Relative to First","Relative to Last"]
      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
    }
    Text {
      Layout.columnSpan: 4
      text: "Remember to Pause"
      font.weight: Font.Bold
      Layout.alignment: Qt.AlignTop | Qt.AlignHCenter
    }
  }
}
