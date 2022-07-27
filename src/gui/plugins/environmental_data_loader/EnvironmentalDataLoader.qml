/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
  columns: 5
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 400
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Text {
    Layout.columnSpan: 1
    id: dataFileText
    color: "dimgrey"
    text: "Data"
  }

  FileDialog {
    Layout.columnSpan: 4
    id: dataFileDialog
    title: "Please choose a data file"
    folder: shortcuts.home
    visible: false
    onAccepted: {
      EnvironmentalDataLoader.SetDataPath(fileDialog.fileUrl)
    }
  }

  Text {
    Layout.columnSpan: 1
    id: timeDimensionText
    color: "dimgrey"
    text: "Time"
  }

  ComboBox {
    Layout.columnSpan: 4
    id: timeDimensionCombo
    Layout.fillWidth: true
    model: EnvironmentalDataLoader.dimensionList
    currentIndex: EnvironmentalDataLoader.timeIndex
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      EnvironmentalDataLoader.SetTimeIndex(currentIndex);
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Data field to be used as time dimension")
  }

  Text {
    Layout.columnSpan: 2
    id: xDimensionText
    color: "dimgrey"
    text: "X"
  }

  ComboBox {
    Layout.columnSpan: 4
    id: xDimensionCombo
    Layout.fillWidth: true
    model: EnvironmentalDataLoader.dimensionList
    currentIndex: EnvironmentalDataLoader.xIndex
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      EnvironmentalDataLoader.SetXIndex(currentIndex);
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Data field to be used as x dimension")
  }

  Text {
    Layout.columnSpan: 2
    id: yDimensionText
    color: "dimgrey"
    text: "Y"
  }

  ComboBox {
    Layout.columnSpan: 4
    id: yDimensionCombo
    Layout.fillWidth: true
    model: EnvironmentalDataLoader.dimensionList
    currentIndex: EnvironmentalDataLoader.yIndex
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      EnvironmentalDataLoader.SetYIndex(currentIndex);
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.teyt: qsTr("Data field to be used as y dimension")
  }

  Text {
    Layout.columnSpan: 2
    id: zDimensionText
    color: "dimgrey"
    text: "Z"
  }

  ComboBox {
    Layout.columnSpan: 4
    id: zDimensionCombo
    Layout.fillWidth: true
    model: EnvironmentalDataLoader.dimensionList
    currentIndex: EnvironmentalDataLoader.zIndex
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      EnvironmentalDataLoader.SetZIndex(currentIndex);
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.teyt: qsTr("Data field to be used as z dimension")
  }

  Button {
    text: "Load"
    Layout.columnSpan: 5
    enabled: EnvironmentalDataLoader.configured
    onClicked: function() {
      EnvironmentalDataLoader.ScheduleUpdate()
    }
  }
}
