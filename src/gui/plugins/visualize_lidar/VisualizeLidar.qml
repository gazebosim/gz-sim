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
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import "qrc:/qml"

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 400
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  property int tooltipDelay: 500
  property int tooltipTimeout: 1000

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: displayVisual
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Display Lidar Visual")
    checked: true
    onClicked: {
      VisualizeLidar.DisplayVisual(checked)
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: displayNonHitting
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Show Non Hitting Rays")
    checked: true
    onClicked: {
      VisualizeLidar.UpdateNonHitting(checked)
    }
  }

  RoundButton {
    Layout.columnSpan: 1
    text: "\u21bb"
    Material.background: Material.primary
    onClicked: {
      combo.currentIndex = 0
      VisualizeLidar.OnRefresh();
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Refresh list of topics publishing laser scans")
  }

  ComboBox {
    Layout.columnSpan: 5
    id: combo
    Layout.fillWidth: true
    model: VisualizeLidar.topicList
    currentIndex: 0
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      VisualizeLidar.OnTopic(textAt(currentIndex));
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Gazebo Transport topics publishing LaserScan messages")
  }

  Text {
    Layout.columnSpan: 2
    id: minRangeText
    color: "dimgrey"
    text: "Min. Range (m)"
  }

  Text{
    Layout.columnSpan: 4
    id: minRangeField
    text: VisualizeLidar.minRange
  }

  Text {
    Layout.columnSpan: 2
    id: maxRangeText
    color: "dimgrey"
    text: "Max. Range (m)"
  }

  Text{
    Layout.columnSpan: 4
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
    Layout.columnSpan: 4
    id: typeCombo
    Layout.fillWidth: true
    currentIndex: 3
    model: ["None", "Rays", "Points", "Triangle Strips" ]
    onCurrentIndexChanged: {
      if (currentIndex < 0) {
        return;
      }
      VisualizeLidar.UpdateType(typeCombo.currentIndex);
    }
  }

  Text {
    Layout.columnSpan: 2
    id: pointSizeText
    color: "dimgrey"
    text: "Point Size"
  }

  GzSpinBox {
    Layout.columnSpan: 2
    id: pointSize
    maximumValue: 1000
    minimumValue: 1
    value: 1
    onEditingFinished: VisualizeLidar.UpdateSize(pointSize.value)
  }
}
