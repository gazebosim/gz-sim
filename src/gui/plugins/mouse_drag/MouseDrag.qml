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
  Layout.minimumHeight: 300
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Maximum value for GzSpinBoxes
  property int maxValue: 1000000

  // Precision of the GzSpinBoxes in decimal places
  property int decimalPlaces: 1

  // Step size of the GzSpinBoxes
  property double step: 10.0

  Text {
    Layout.columnSpan: 8
    id: rotationText
    text: qsTr("Rotation (Ctrl+Left-Click)")
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: rotStiffText
    text: qsTr("Rotation stiffness")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: rotStiffness
    maximumValue: maxValue
    minimumValue: 0
    value: MouseDrag.rotStiffness
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: MouseDrag.rotStiffness = rotStiffness.value
  }

  Text {
    Layout.columnSpan: 8
    id: translationText
    text: qsTr("Translation (Ctrl+Right-Click)")
  }

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: posStiffText
    text: qsTr("Position stiffness")
  }

  GzSpinBox {
    Layout.columnSpan: 6
    Layout.fillWidth: true
    id: posStiffness
    maximumValue: maxValue
    minimumValue: 0
    value: MouseDrag.posStiffness
    decimals: decimalPlaces
    stepSize: step
    onValueChanged: MouseDrag.posStiffness = posStiffness.value
  }

  Switch {
    Layout.columnSpan: 8
    objectName: "switchCOM"
    text: qsTr("Apply force to center of mass")
    onToggled: {
      MouseDrag.OnSwitchCOM(checked);
    }
  }
}
