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
  Layout.minimumWidth: 250
  Layout.minimumHeight: 300
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
    id: visualizeLidar
    Layout.columnSpan: 4
    text: qsTr("Show Non Hitting Rays")
    checked: true
    onClicked: {
      VisualizeLidar.UpdateNonHitting(checked)
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
    id: topicNameText
    color: "dimgrey"
    text: "Topic Name"
  }

  TextField {
    id: topicNameField
    text: VisualizeLidar.topicName
    onEditingFinished: VisualizeLidar.topicName = topicNameField.text
  }

  // Right spacer
  Item {
    Layout.columnSpan: 1
    Layout.rowSpan: 15
    Layout.fillWidth: true
  }

  Text {
    Layout.columnSpan: 2
    id: minRangeText
    color: "dimgrey"
    text: "Minimum Range (m)"
  }

  Text{
    id: minRangeField
    text: VisualizeLidar.minRange
  }

  Text {
    Layout.columnSpan: 2
    id: maxRangeText
    color: "dimgrey"
    text: "Maximum Range (m)"
  }

  Text{
    id: maxRangeField
    text: VisualizeLidar.maxRange
  }

  Text {
    Layout.columnSpan: 2
    id: updatePeriodText
    color: "dimgrey"
    text: "Visual Type"
  }

  ComboBox {
    id: typeCombo
    Layout.fillWidth: true
    currentIndex: 3
    model: ["None", "Rays", "Points", "Triangles" ]
    onCurrentIndexChanged: {
      if (currentIndex < 0) {
        return;
      }
      VisualizeLidar.UpdateType(typeCombo.currentIndex);
    }
  }
}
