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

ColumnLayout {
  Layout.minimumWidth: 320
  Layout.minimumHeight: 530
  anchors.fill: parent

  ToolBar {
    Layout.fillWidth: true
    background: Rectangle {
      color: "transparent"
    }

    ButtonGroup {
      id: group
    }

    GridLayout {
      id: views
      anchors.horizontalCenter: parent.horizontalCenter
      columns: 4
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
    }
  }

  // Projection
  ComboBox {
    currentIndex: 0
    model: ListModel {
        id: controller
        ListElement {text: "Orbit View Control"}
        ListElement {text: "Orthographic View Control"}
    }
    Layout.fillWidth: true
    Layout.minimumWidth: 280
    Layout.margins: 10
    onCurrentIndexChanged: {
        ViewAngle.OnViewControl(controller.get(currentIndex).text)
    }
  }

  // Set camera pose
  Text {
    text: "Camera Pose"
    Layout.fillWidth: true
    color: Material.Grey
    leftPadding: 5
    font.bold: true
  }

  GridLayout {
    width: parent.width
    columns: 6

    Text {
      text: "X (m)"
      color: "dimgrey"
      Layout.row: 0
      Layout.column: 0
      leftPadding: 5
    }
    GzSpinBox {
      id: x
      Layout.fillWidth: true
      Layout.row: 0
      Layout.column: 1
      value: ViewAngle.camPose[0]
      maximumValue: Number.MAX_VALUE
      minimumValue: -Number.MAX_VALUE
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Y (m)"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 0
      leftPadding: 5
    }
    GzSpinBox {
      id: y
      Layout.fillWidth: true
      Layout.row: 1
      Layout.column: 1
      value: ViewAngle.camPose[1]
      maximumValue: Number.MAX_VALUE
      minimumValue: -Number.MAX_VALUE
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Z (m)"
      color: "dimgrey"
      Layout.row: 2
      Layout.column: 0
      leftPadding: 5
    }
    GzSpinBox {
      id: z
      Layout.fillWidth: true
      Layout.row: 2
      Layout.column: 1
      value: ViewAngle.camPose[2]
      maximumValue: Number.MAX_VALUE
      minimumValue: -Number.MAX_VALUE
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }

    Text {
      text: "Roll (rad)"
      color: "dimgrey"
      Layout.row: 0
      Layout.column: 2
      leftPadding: 5
    }
    GzSpinBox {
      id: roll
      Layout.fillWidth: true
      Layout.row: 0
      Layout.column: 3
      value: ViewAngle.camPose[3]
      maximumValue: 6.28
      minimumValue: -6.28
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Pitch (rad)"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 2
      leftPadding: 5
    }
    GzSpinBox {
      id: pitch
      Layout.fillWidth: true
      Layout.row: 1
      Layout.column: 3
      value: ViewAngle.camPose[4]
      maximumValue: 6.28
      minimumValue: -6.28
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
    Text {
      text: "Yaw (rad)"
      color: "dimgrey"
      Layout.row: 2
      Layout.column: 2
      leftPadding: 5
    }
    GzSpinBox {
      id: yaw
      Layout.fillWidth: true
      Layout.row: 2
      Layout.column: 3
      value: ViewAngle.camPose[5]
      maximumValue: 6.28
      minimumValue: -6.28
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
    }
  }

  // Set camera's near/far clipping distance
  Text {
    text: "Camera's clipping plane distance"
    Layout.fillWidth: true
    color: Material.Grey
    leftPadding: 5
    topPadding: 10
    font.bold: true
  }

  GridLayout {
    width: parent.width
    columns: 4

    Text {
      text: "Near (m)"
      color: "dimgrey"
      Layout.row: 0
      Layout.column: 0
      leftPadding: 5
    }
    GzSpinBox {
      id: nearClip
      Layout.fillWidth: true
      Layout.row: 0
      Layout.column: 1
      value: ViewAngle.camClipDist[0]
      maximumValue: farClip.value
      minimumValue: 0.000001
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamClipDist(nearClip.value, farClip.value)
    }
    Text {
      text: "Far (m)"
      color: "dimgrey"
      Layout.row: 0
      Layout.column: 2
      leftPadding: 5
    }
    GzSpinBox {
      id: farClip
      Layout.fillWidth: true
      Layout.row: 0
      Layout.column: 3
      value: ViewAngle.camClipDist[1]
      maximumValue: Number.MAX_VALUE
      minimumValue: nearClip.value
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetCamClipDist(nearClip.value, farClip.value)
    }
  }

  // Bottom spacer
  Item {
    width: 10
    Layout.fillHeight: true
  }
}
