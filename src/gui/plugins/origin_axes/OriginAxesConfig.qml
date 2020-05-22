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
  Layout.minimumHeight: 200
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
      OriginAxesConfig.OnShow(checked)
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
    onEditingFinished: OriginAxesConfig.UpdateLength(axesLength.value)
    Layout.fillWidth: true
  }
  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 1
    Layout.fillWidth: true
  }
}
