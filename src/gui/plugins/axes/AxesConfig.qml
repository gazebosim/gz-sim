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
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 4
  columnSpacing: 10
  Layout.minimumWidth: 200
  Layout.minimumHeight: 500
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    Layout.columnSpan: 2
    text: qsTr("Show/Hide Axes")
    checked: true
    onClicked: {
      AxesConfig.OnShow(checked)
    }
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  ComboBox {
    Layout.alignment: Qt.AlignHCenter
    Layout.columnSpan: 2
    objectName: "ComboBoxEntities"
    model: AxesConfig.comboList
    onActivated: {
      AxesConfig.onCurrentIndexChanged(index)
    }
    onAccepted: {
         if (editableCombo.find(currentText) === -1) {
             model.append({text: editText})
             currentIndex = editableCombo.find(editText)
         }
     }
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    Layout.columnSpan: 2
    text: qsTr("Lines/Arrow Axes")
    checked: true
    onClicked: {
      AxesConfig.OnTypeAxes(checked)
    }
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }


  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  Text {
    Layout.columnSpan: 1
    color: "dimgrey"
    text: "Size(meters):"
    Layout.fillWidth: true
  }

  IgnSpinBox {
    Layout.columnSpan: 1
    id: axesLength
    maximumValue: 100
    minimumValue: 1
    value: 1
    decimals: 2
    stepSize: 0.5
    onEditingFinished: AxesConfig.UpdateLength(axesLength.value)
    Layout.fillWidth: true
  }
  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }

  Text {
    Layout.columnSpan: 2
    id: cartesian
    color: "dimgrey"
    font.bold: true
    text: "Position (m)"
  }

  Text {
    Layout.columnSpan: 2
    id: principal
    text: "Rotation (rad)"
    color: "dimgrey"
    font.bold: true
  }

  Text {
    text: "X"
    color: "dimgrey"
  }

  IgnSpinBox {
    id: x
    value: 0.00
    maximumValue: 1000.00
    minimumValue: -1000.00
    decimals: 2
    stepSize: 0.01
    onEditingFinished: AxesConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
  }

  Text {
    text: "Roll"
    color: "dimgrey"
  }

  IgnSpinBox {
    id: roll
    maximumValue: 6.28
    minimumValue: 0.00
    value: 0.00
    decimals: 2
    stepSize: 0.01
    onEditingFinished: AxesConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
  }

  Text {
    text: "Y"
    color: "dimgrey"
  }

  IgnSpinBox {
    id: y
    value: 0.00
    maximumValue: 1000.00
    minimumValue: -1000.00
    decimals: 2
    stepSize: 0.01
    onEditingFinished: AxesConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
  }

  Text {
    text: "Pitch"
    color: "dimgrey"
  }

  IgnSpinBox {
    id: pitch
    maximumValue: 6.28
    minimumValue: 0.00
    value: 0.00
    decimals: 2
    stepSize: 0.01
    onEditingFinished: AxesConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
  }

  Text {
    text: "Z"
    color: "dimgrey"
  }

  IgnSpinBox {
    id: z
    value: 0.00
    maximumValue: 1000.00
    minimumValue: -1000.00
    decimals: 2
    stepSize: 0.01
    onEditingFinished: AxesConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
  }

  Text {
    text: "Yaw"
    color: "dimgrey"
  }

  IgnSpinBox {
    id: yaw
    maximumValue: 6.28
    minimumValue: 0.00
    value: 0.00
    decimals: 2
    stepSize: 0.01
    onEditingFinished: AxesConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
  }
}
