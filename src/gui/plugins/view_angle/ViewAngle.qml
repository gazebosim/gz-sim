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
  Layout.minimumHeight: 650
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
    currentIndex: ViewAngle.viewControlIndex
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

  // view control sensitivity
  GridLayout {
    Layout.fillWidth: true
    Layout.margins: 10
    columns: 2

    Label {
      id: viewControlSensitivityLabel
      text: "View control sensitivity"
    }
    GzSpinBox {
      id: viewControlSensitivitySpinBox
      Layout.fillWidth: true
      value: 1.0
      maximumValue: 10.0
      minimumValue: 0.01
      decimals: 2
      stepSize: 0.1
      onEditingFinished:{
        ViewAngle.OnViewControlSensitivity(value)
      }
    }
  }

  // toggle view control reference visual
  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: displayVisual
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Display View Control Reference Visual")
    checked: true
    onClicked: {
      ViewAngle.OnViewControlReferenceVisual(checked)
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

  GzPose {
    y: 30
    width: parent.width
    Layout.fillWidth: true
    readOnly: false
    xValue: ViewAngle.camPose[0]
    yValue: ViewAngle.camPose[1]
    zValue: ViewAngle.camPose[2]
    rollValue: ViewAngle.camPose[3]
    pitchValue: ViewAngle.camPose[4]
    yawValue: ViewAngle.camPose[5]
    onGzPoseSet: {
      // _x, _y, _z, _roll, _pitch, _yaw are parameters of signal gzPoseSet
      // from gz-gui GzPose.qml
      ViewAngle.SetCamPose(_x, _y, _z, _roll, _pitch, _yaw)
    }
    expand: true
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

  // Set camera's horizontal FOV
  Text {
    text: "Horizontal FOV"
    Layout.fillWidth: true
    color: Material.Grey
    leftPadding: 5
    topPadding: 10
    font.bold: true
  }

  GridLayout {
    width: parent.width
    columns: 2

    Text {
      text: "HorizontalFOV (rads)"
      color: "dimgrey"
      Layout.row: 0
      Layout.column: 0
      leftPadding: 5
    }
    GzSpinBox {
      id: horizontalFOV
      Layout.fillWidth: true
      Layout.row: 0
      Layout.column: 1
      value: ViewAngle.horizontalFOV
      maximumValue: 3.14159
      minimumValue: 0.000001
      decimals: 6
      stepSize: 0.01
      onEditingFinished: ViewAngle.SetHorizontalFOV(horizontalFOV.value)
    }
  }

  // Bottom spacer
  Item {
    width: 10
    Layout.fillHeight: true
  }
}
