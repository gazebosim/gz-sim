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
  columns: 8
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 500
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Label {
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: dataFileText
    color: "dimgrey"
    text: qsTr("Data file path")
  }

  RowLayout {
    Layout.column: 2
    Layout.columnSpan: 6

    TextField {
      id: dataFilePathInput
      Layout.fillWidth: true
      text: EnvironmentLoader.dataPath
      placeholderText: qsTr("Path to data file")
      onEditingFinished: {
        EnvironmentLoader.dataPath = text
      }
    }

    Button {
      id: browseDataFile
      Layout.preferredWidth: 20
      display: AbstractButton.IconOnly
      text: EnvironmentLoader.dataFileName
      onClicked: dataFileDialog.open()
      icon.source: "qrc:/Gazebo/images/chevron-right.svg"
      ToolTip.visible: hovered
      ToolTip.text: qsTr("Browse files...")
    }
  }

  FileDialog {
    Layout.columnSpan: 8
    id: dataFileDialog
    title: qsTr("Please choose a data file")
    folder: shortcuts.home
    visible: false
    onAccepted: {
      EnvironmentLoader.SetDataUrl(dataFileDialog.fileUrl)
    }
    onRejected: {
    }
    Component.onCompleted: visible = false
  }

  Label {
    Layout.row: 1
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: timeDimensionText
    color: "dimgrey"
    text: qsTr("Time dimension")
  }

  ComboBox {
    Layout.row: 1
    Layout.column: 2
    Layout.columnSpan: 6
    id: timeDimensionCombo
    Layout.fillWidth: true
    enabled: EnvironmentLoader.configured
    model: EnvironmentLoader.dimensionList
    currentIndex: EnvironmentLoader.timeIndex
    onCurrentIndexChanged: {
      EnvironmentLoader.timeIndex = currentIndex
    }
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Data field to be used as time dimension")
  }

  Label {
    Layout.row: 2
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: xDimensionText
    color: "dimgrey"
    text: qsTr("X dimension")
  }

  ComboBox {
    Layout.row: 2
    Layout.column: 2
    Layout.columnSpan: 6
    id: xDimensionCombo
    Layout.fillWidth: true
    enabled: EnvironmentLoader.configured
    model: EnvironmentLoader.dimensionList
    currentIndex: EnvironmentLoader.xIndex
    onCurrentIndexChanged: {
      EnvironmentLoader.xIndex = currentIndex
    }
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Data field to be used as x dimension")
  }

  Label {
    Layout.row: 3
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: yDimensionText
    color: "dimgrey"
    text: qsTr("Y dimension")
  }

  ComboBox {
    Layout.row: 3
    Layout.column: 2
    Layout.columnSpan: 6
    id: yDimensionCombo
    Layout.fillWidth: true
    enabled: EnvironmentLoader.configured
    model: EnvironmentLoader.dimensionList
    currentIndex: EnvironmentLoader.yIndex
    onCurrentIndexChanged: {
      EnvironmentLoader.yIndex = currentIndex
    }
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Data field to be used as y dimension")
  }

  Label {
    Layout.row: 4
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: zDimensionText
    color: "dimgrey"
    text: qsTr("Z dimension")
  }

  ComboBox {
    Layout.row: 4
    Layout.column: 2
    Layout.columnSpan: 6
    id: zDimensionCombo
    Layout.fillWidth: true
    enabled: EnvironmentLoader.configured
    model: EnvironmentLoader.dimensionList
    currentIndex: EnvironmentLoader.zIndex
    onCurrentIndexChanged: {
      EnvironmentLoader.zIndex = currentIndex
    }
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Data field to be used as z dimension")
  }

  Label {
    Layout.row: 5
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: refText
    color: "dimgrey"
    text: qsTr("Reference")
  }

  ComboBox {
    Layout.row: 5
    Layout.column: 2
    Layout.columnSpan: 6
    id: referenceCombo
    Layout.fillWidth: true
    enabled: EnvironmentLoader.configured
    model: EnvironmentLoader.referenceList
    onCurrentTextChanged: {
      EnvironmentLoader.reference = currentText
    }
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Reference for spatial dimensions")
  }

  Label {
    Layout.row: 6
    Layout.columnSpan: 1
    horizontalAlignment: Text.AlignRight
    id: unitText
    color: "dimgrey"
    text: qsTr("Units")
  }

  ComboBox {
    Layout.row: 6
    Layout.column: 2
    Layout.columnSpan: 6
    id: unitsConversion
    Layout.fillWidth: true
    enabled: referenceCombo.currentIndex == 2
    model: EnvironmentLoader.unitList
    onCurrentTextChanged: {
      EnvironmentLoader.unit = currentText
    }
    ToolTip.visible: hovered
    ToolTip.text: qsTr("Units")
  }


  Button {
    text: qsTr("Load")
    Layout.row: 7
    Layout.columnSpan: 8
    Layout.fillWidth: true
    enabled: EnvironmentLoader.configured
    onClicked: function() {
      EnvironmentLoader.ScheduleLoad()
    }
  }
}
