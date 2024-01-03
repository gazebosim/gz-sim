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
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 8
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 750
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Maximum value for GzSpinBoxes
  property int maxValue: 1000000

  // Minimum value for GzSpinBoxes
  property int minValue: -1000000

  // Precision of the GzSpinBoxes in decimal places
  property int decimalPlaces: 2

  // Step size of the force and torque
  property double step: 100.0

  // Step size of the offset
  property double stepOffset: 1.0

  Label {
    Layout.columnSpan: 8
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    id: frameText
    text: "Forces and torques are given in link-fixed frame."
  }

  Label {
    Layout.columnSpan: 8
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    id: rotText
    text: "Click on an arrow to toggle its rotation tool."
  }

  Text {
    Layout.columnSpan: 2
    id: modelText
    color: "black"
    text: qsTr("Model:")
  }

  Text {
    Layout.columnSpan: 6
    id: modelName
    color: "black"
    text: ApplyForceTorque.modelName
  }

  Text {
    Layout.columnSpan: 2
    id: linkText
    color: "black"
    text: qsTr("Link:")
  }

  ComboBox {
    Layout.columnSpan: 6
    id: linkCombo
    Layout.fillWidth: true
    model: ApplyForceTorque.linkNameList
    currentIndex: ApplyForceTorque.linkIndex
    onCurrentIndexChanged: {
      ApplyForceTorque.linkIndex = currentIndex
    }
  }

  // Force and offset
  Text {
    Layout.columnSpan: 4
    id: forceText
    color: "black"
    text: qsTr("Force:")
  }

  Text {
    Layout.columnSpan: 4
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    id: offsetText
    color: "black"
    text: qsTr("Offset (from COM):")
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceXText
    color: "black"
    text: qsTr("X (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceX
    maximumValue: maxValue
    minimumValue: minValue
    value: ApplyForceTorque.force.x
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.force.x = forceX.value
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: offsetXText
    color: "black"
    text: qsTr("X (m)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: offsetX
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: stepOffset
    onValueChanged: ApplyForceTorque.UpdateOffset(
      offsetX.value, offsetY.value, offsetZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceYText
    color: "black"
    text: qsTr("Y (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceY
    maximumValue: maxValue
    minimumValue: minValue
    value: ApplyForceTorque.force.y
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.force.y = forceY.value
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: offsetYText
    color: "black"
    text: qsTr("Y (m)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: offsetY
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: stepOffset
    onValueChanged: ApplyForceTorque.UpdateOffset(
      offsetX.value, offsetY.value, offsetZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceZText
    color: "black"
    text: qsTr("Z (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceZ
    maximumValue: maxValue
    minimumValue: minValue
    value: ApplyForceTorque.force.z
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.force.z = forceZ.value
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: offsetZText
    color: "black"
    text: qsTr("Z (m)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: offsetZ
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: stepOffset
    onValueChanged: ApplyForceTorque.UpdateOffset(
      offsetX.value, offsetY.value, offsetZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceMagText
    color: "black"
    text: qsTr("Mag. (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: forceMag
    maximumValue: maxValue
    minimumValue: 0
    value: ApplyForceTorque.forceMag
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.forceMag = forceMag.value
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
  Text {
    Layout.columnSpan: 8
    id: torqueText
    color: "black"
    text: qsTr("Torque:")
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueXText
    color: "black"
    text: qsTr("X (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueX
    maximumValue: maxValue
    minimumValue: minValue
    value: ApplyForceTorque.torque.x
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.torque.x = torqueX.value
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueYText
    color: "black"
    text: qsTr("Y (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueY
    maximumValue: maxValue
    minimumValue: minValue
    value: ApplyForceTorque.torque.y
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.torque.y = torqueY.value
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueZText
    color: "black"
    text: qsTr("Z (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueZ
    maximumValue: maxValue
    minimumValue: minValue
    value: ApplyForceTorque.torque.z
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.torque.z = torqueZ.value
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: torqueMagText
    color: "black"
    text: qsTr("Mag. (N.m)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: torqueMag
    maximumValue: maxValue
    minimumValue: 0
    value: ApplyForceTorque.torqueMag
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.torqueMag = torqueMag.value
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
