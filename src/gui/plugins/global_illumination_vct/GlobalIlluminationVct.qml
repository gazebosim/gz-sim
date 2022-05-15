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
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.3
//import QtQuick.Controls.Styles 1.4
import "qrc:/qml"

/*
import "qrc:/ComponentInspector"
import "qrc:/"*/

GridLayout {
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 400
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: displayVisual
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Enabled")
    checked: GlobalIlluminationVct.enabled
    onClicked: {
        GlobalIlluminationVct.enabled = checked
    }
  }

  Text {
    Layout.columnSpan: 4
    id: bounceCountStr
    color: "dimgrey"
    text: qsTr("BounceCount")
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: bounceCount
    value: writableSpin.activeFocus ? writableSpin.value : numberValue
    minimumValue: 0
    maximumValue: 16
    decimals: 1
    onEditingFinished: {
      sendLight()
    }
  }

  Text {
    Layout.columnSpan: 6
    id: resolutionStr
    color: "dimgrey"
    text: qsTr("Resolution")
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinX
    value: resolutionSpinX.activeFocus ? resolutionSpinX.value : numberValue
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      // VisualizeLidar.UpdateResolution(0,resolutionSpinX.value)
    }
  }
  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinY
    value: resolutionSpinY.activeFocus ? resolutionSpinY.value : numberValue
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      // VisualizeLidar.UpdateResolution(1,resolutionSpinY.value)
    }
  }
  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinZ
    value: resolutionSpinZ.activeFocus ? resolutionSpinZ.value : numberValue
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      // VisualizeLidar.UpdateResolution(2,resolutionSpinZ.value)
    }
  }

  Text {
    Layout.columnSpan: 6
    id: octantCountStr
    color: "dimgrey"
    text: qsTr("OctantCount")
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountX
    value: octantCountX.activeFocus ? octantCountX.value : numberValue
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      sendLight()
    }
  }
  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountY
    value: octantCountY.activeFocus ? octantCountY.value : numberValue
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      sendLight()
    }
  }
  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountZ
    value: octantCountZ.activeFocus ? octantCountZ.value : numberValue
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      sendLight()
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: conserveMemory
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Conserve Memory")
    checked: false
    onClicked: {
        // VisualizeLidar.DisplayVisual(checked)
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: highQuality
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("High Quality")
    checked: true
    onClicked: {
        // VisualizeLidar.DisplayVisual(checked)
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: anisotropic
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Anisotropic")
    checked: true
    onClicked: {
        // VisualizeLidar.DisplayVisual(checked)
    }
  }

  Text {
    Layout.columnSpan: 4
    id: thinWallCounterStr
    color: "dimgrey"
    text: qsTr("ThinWallCounter")
  }

  IgnSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: thinWallCounter
    value: thinWallCounter.activeFocus ? thinWallCounter.value : numberValue
    minimumValue: 0
    maximumValue: 3
    onEditingFinished: {
      sendLight()
    }
  }

  Text {
    Layout.columnSpan: 2
    id: debugVisualizationStr
    color: "dimgrey"
    text: qsTr("DebugVisualization")
  }

  ComboBox {
    Layout.columnSpan: 4
    id: debugVisualization
    Layout.fillWidth: true
    currentIndex: 3
    model: ["Albedo", "Normal", "Emissive", "Lighting", "None"]
    onCurrentIndexChanged: {
      if (currentIndex < 0) {
        return;
      }
      /*GlobalIlluminationVct.UpdateDebugVisualizationMode(
            typeCombo.currentIndex);*/
    }
  }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}D{i:1}D{i:2}D{i:3}D{i:4}D{i:5}D{i:6}D{i:7}
D{i:8}D{i:9}D{i:10}D{i:11}D{i:12}D{i:13}D{i:14}D{i:15}D{i:16}D{i:17}D{i:18}
}
##^##*/
