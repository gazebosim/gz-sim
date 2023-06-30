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
  Layout.minimumHeight: 650
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Maximum value for GzSpinBoxes
  property int maxValue: 1000000

  // Minimum value for GzSpinBoxes
  property int minValue: -1000000

  // Precision of the GzSpinBoxes in decimal places
  property int decimalPlaces: 2

  // Step size of the GzSpinBoxes
  property double step: 1.0

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
    model: ApplyForceTorque.linkNameList
    currentIndex: ApplyForceTorque.linkIndex
    onCurrentIndexChanged: {
      ApplyForceTorque.linkIndex = currentIndex
    }
  }

  // Force
  Text {
    Layout.columnSpan: 8
    id: forceText
    color: "dimgrey"
    text: qsTr("Force (applied to link origin):")
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceXText
    color: "dimgrey"
    text: qsTr("X (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: forceX
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.UpdateForce(
      forceX.value, forceY.value, forceZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceYText
    color: "dimgrey"
    text: qsTr("Y (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: forceY
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.UpdateForce(
      forceX.value,forceY.value, forceZ.value)
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: forceZText
    color: "dimgrey"
    text: qsTr("Z (N)")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: forceZ
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: ApplyForceTorque.UpdateForce(
      forceX.value, forceY.value, forceZ.value)
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
    color: "dimgrey"
    text: qsTr("Torque:")
  }

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
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: step
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
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: step
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
    maximumValue: maxValue
    minimumValue: minValue
    value: 0
    decimals: decimalPlaces
    stepSize: step
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
