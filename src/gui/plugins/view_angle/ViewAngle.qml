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
import "qrc:/qml"

ToolBar {
  Layout.minimumWidth: 420
  Layout.minimumHeight: 260

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  GridLayout {
    columns: 8
    ToolButton {
      id: top
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the top"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 0
      Layout.column: 1
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_top.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, -1)
      }
    }
    ToolButton {
      id: home
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Reset View Angle"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 0
      Layout.column: 3
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_home.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 0)
      }
    }
    ToolButton {
      id: left
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the left"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 0
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_left.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 1, 0)
      }
    }
    ToolButton {
      id: front
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the front"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 1
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_front.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(-1, 0, 0)
      }
    }
    ToolButton {
      id: right
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the right"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 2
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_right.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, -1, 0)
      }
    }
    ToolButton {
      id: back
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the back"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 3
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_back.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(1, 0, 0)
      }
    }
    ToolButton {
      id: bottom
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the bottom"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 2
      Layout.column: 1
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_bottom.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 1)
      }
    }

    // set camera pose
    // xyz
    Text {
      Layout.columnSpan: 2
      Layout.row: 0
      Layout.column: 5
      color: "dimgrey"
      font.bold: true
      text: "Position (m)"
    }

    Text {
      text: "X"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 5
    }
    IgnSpinBox {
      id: x
      Layout.row: 1
      Layout.column: 6
      value: ViewAngle.camPose[0]
      maximumValue: 1000.00
      minimumValue: -1000.00
      decimals: 2
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Y"
      color: "dimgrey"
      Layout.row: 2
      Layout.column: 5
    }
    IgnSpinBox {
      id: y
      Layout.row: 2
      Layout.column: 6
      value: ViewAngle.camPose[1]
      maximumValue: 1000.00
      minimumValue: -1000.00
      decimals: 2
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Z"
      color: "dimgrey"
      Layout.row: 3
      Layout.column: 5
    }
    IgnSpinBox {
      id: z
      Layout.row: 3
      Layout.column: 6
      value: ViewAngle.camPose[2]
      maximumValue: 1000.00
      minimumValue: -1000.00
      decimals: 2
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }

    // rpy
    Text {
      Layout.columnSpan: 2
      Layout.row: 0
      Layout.column: 7
      color: "dimgrey"
      font.bold: true
      text: "Rotation (rad)"
    }

    Text {
      text: "Roll"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 7
    }
    IgnSpinBox {
      id: roll
      Layout.row: 1
      Layout.column: 8
      value: ViewAngle.camPose[3]
      maximumValue: 6.28
      minimumValue: 0.00
      decimals: 2
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Pitch"
      color: "dimgrey"
      Layout.row: 2
      Layout.column: 7
    }
    IgnSpinBox {
      id: pitch
      Layout.row: 2
      Layout.column: 8
      value: ViewAngle.camPose[4]
      maximumValue: 6.28
      minimumValue: 0.00
      decimals: 2
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Yaw"
      color: "dimgrey"
      Layout.row: 3
      Layout.column: 7
    }
    IgnSpinBox {
      id: yaw
      Layout.row: 3
      Layout.column: 8
      value: ViewAngle.camPose[5]
      maximumValue: 6.28
      minimumValue: 0.00
      decimals: 2
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
  }
}
