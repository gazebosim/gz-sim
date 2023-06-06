/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
import QtQuick.Dialogs 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 8
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 600
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Text {
    Layout.columnSpan: 2
    id: modelText
    color: "dimgrey"
    text: qsTr("Model:")
  }

  Text {
    Layout.columnSpan: 6
    id: modelName
    color: "dimgrey"
    text: ApplyForceTorque.modelName
  }

  Text {
    Layout.columnSpan: 2
    id: linkText
    color: "dimgrey"
    text: qsTr("Link:")
  }

  ComboBox {
    Layout.columnSpan: 6
    id: linkCombo
    Layout.fillWidth: true
    // enabled: ApplyForceTorque.configured
    model: ApplyForceTorque.linkNameList
    currentIndex: ApplyForceTorque.linkIndex
    onCurrentIndexChanged: {
      ApplyForceTorque.linkIndex = currentIndex
    }
    // ToolTip.visible: hovered
    // ToolTip.text: qsTr("Link to which the wrench should be applied")
  }

  // Force
  Text {
    Layout.columnSpan: 4
    id: forceText
    color: "dimgrey"
    text: qsTr("Force")
  }
  
  Text {
    Layout.columnSpan: 4
    horizontalAlignment: Text.AlignHCenter
    id: torqueText
    color: "dimgrey"
    text: qsTr("Application point")
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceXText
    color: "dimgrey"
    text: qsTr("X (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceX
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateForce(
      forceX.value, forceY.value, forceZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: offsetXText
    color: "dimgrey"
    text: qsTr("X (m)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: offsetX
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateOffset(
      offsetX.value, offsetY.value, offsetZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceYText
    color: "dimgrey"
    text: qsTr("Y (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceY
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateForce(
      forceX.value,forceY.value, forceZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: offsetYText
    color: "dimgrey"
    text: qsTr("Y (m)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: offsetY
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateOffset(
      offsetX.value, offsetY.value, offsetZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceZText
    color: "dimgrey"
    text: qsTr("Z (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceZ
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateForce(
      forceX.value, forceY.value, forceZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: offsetZText
    color: "dimgrey"
    text: qsTr("Z (m)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: offsetZ
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateOffset(
      offsetX.value, offsetY.value, offsetZ.value)
  }

  Button {
    text: qsTr("Apply Force")
    Layout.columnSpan: 8
    Layout.fillWidth: true
    onClicked: function() {
      ApplyForceTorque.ApplyForce()
    }
  }

  // Torque
  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueXText
    color: "dimgrey"
    text: qsTr("X (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueX
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateTorque(
      torqueX.value,torqueY.value, torqueZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueYText
    color: "dimgrey"
    text: qsTr("Y (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueY
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateTorque(
      torqueX.value,torqueY.value, torqueZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueZText
    color: "dimgrey"
    text: qsTr("Z (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueZ
    maximumValue: 1000000
    minimumValue: -1000000
    value: 0
    decimals: 2
    stepSize: 1.0
    onValueChanged: ApplyForceTorque.UpdateTorque(
      torqueX.value,torqueY.value, torqueZ.value)
  }

  Button {
    text: qsTr("Apply Torque")
    Layout.columnSpan: 8
    Layout.fillWidth: true
    onClicked: function() {
      ApplyForceTorque.ApplyTorque()
    }
  }

  Button {
    text: qsTr("Apply All")
    Layout.columnSpan: 8
    Layout.fillWidth: true
    onClicked: function() {
      ApplyForceTorque.ApplyAll()
    }
  }
}