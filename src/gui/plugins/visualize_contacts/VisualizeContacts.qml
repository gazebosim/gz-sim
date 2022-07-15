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
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 250
  Layout.minimumHeight: 400
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  // Left spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: visualizeContacts
    Layout.columnSpan: 4
    text: qsTr("Show Contacts")
    checked: false
    onClicked: {
      VisualizeContacts.OnVisualize(checked)
    }
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  Text {
    Layout.columnSpan: 2
    id: radiusText
    color: "dimgrey"
    text: "Radius (m)"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: radius
    maximumValue: 2.00
    minimumValue: 0.01
    value: 0.10
    decimals: 2
    stepSize: 0.05
    onEditingFinished: VisualizeContacts.UpdateRadius(radius.value)
  }

  Text {
    Layout.columnSpan: 2
    id: updatePeriodText
    color: "dimgrey"
    text: "Update period (ms)"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: updatePeriod
    maximumValue: 2000
    minimumValue: 10
    value: 200
    decimals: 0
    stepSize: 50
    onEditingFinished: VisualizeContacts.UpdatePeriod(updatePeriod.value)
  }

  // Bottom spacer
  Item {
   Layout.columnSpan: 4
   Layout.fillHeight: true
 }
}
