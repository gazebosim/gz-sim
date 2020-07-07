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
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 200
  Layout.minimumHeight: 200
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
    text: qsTr("Show/Hide Contacts")
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
    Layout.columnSpan: 4
    text: "Markers size"
    color: "dimgrey"
    font.bold: true
  }

  Text {
    Layout.columnSpan: 2
    id: radiusText
    color: "dimgrey"
    text: "Radius"
  }

  IgnSpinBox {
    Layout.columnSpan: 2
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
    id: lengthText
    color: "dimgrey"
    text: "Length"
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    id: length
    maximumValue: 2.00
    minimumValue: 0.01
    value: 0.40
    decimals: 2
    stepSize: 0.05
    onEditingFinished: VisualizeContacts.UpdateLength(length.value)
  }
}