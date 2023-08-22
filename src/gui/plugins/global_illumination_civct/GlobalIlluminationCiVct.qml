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
import "qrc:/qml"

GridLayout {
  id: mainGridLayout
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 800
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  property var cascades: []

  function addCascade() {
    var cascade = GlobalIlluminationCiVct.AddCascade()
    var cascadeComponent = Qt.createComponent("CiVctCascadePrivate.qml");
    var cascadeObj = cascadeComponent.createObject(mainGridLayout,
                                                   { "cascadePtr":cascade });
    cascades.push(cascadeObj)
  }

  Connections {
      target: GlobalIlluminationCiVct
      function onQmlAddCascade() {
        mainGridLayout.addCascade()
      }

      function onQmlAddCascade2(_resX, _resY, _resZ, _octX, _octY, _octZ,
                                _ahsX, _ahsY, _ahsZ, _thinWallCounter) {
        var cascade = GlobalIlluminationCiVct.AddCascade()
        var cascadeComponent = Qt.createComponent("CiVctCascadePrivate.qml");
        cascade.resolutionX = _resX
        cascade.resolutionY = _resY
        cascade.resolutionZ = _resZ
        cascade.octantCountX = _octX
        cascade.octantCountY = _octY
        cascade.octantCountZ = _octZ
        cascade.areaHalfSizeX = _ahsX
        cascade.areaHalfSizeY = _ahsY
        cascade.areaHalfSizeZ = _ahsZ
        var cascadeObj =
          cascadeComponent.createObject(mainGridLayout,
                                        { "cascadePtr":cascade });
        cascades.push(cascadeObj)
      }
  }

  Button {
    id: removeCascade
    text: qsTr("Remove Cascade")
    enabled: GlobalIlluminationCiVct.cascadesEditable
    Layout.columnSpan: 3
    Layout.fillWidth: true
    onClicked: {
      if(cascades.length > 0) {
        //mainGridLayout.height = 400 + 400 * (cascades.length + 1)
        cascades[cascades.length - 1].destroy()
        cascades.pop();
        GlobalIlluminationCiVct.PopCascade()
      }
    }
  }
  Button {
    id: addCascade
    text: qsTr("Add Cascade")
    enabled: GlobalIlluminationCiVct.cascadesEditable
    Layout.columnSpan: 3
    Layout.fillWidth: true
    onClicked: {
      mainGridLayout.addCascade()
      }
    }

  Button {
    id: resetCascades
    text: qsTr("Reset Cascades")
    enabled: !GlobalIlluminationCiVct.cascadesEditable
    Layout.columnSpan: 6
    Layout.fillWidth: true
    onClicked: {
      GlobalIlluminationCiVct.ResetCascades()
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: displayVisual
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Enabled")
    checked: GlobalIlluminationCiVct.enabled
    onToggled: {
      GlobalIlluminationCiVct.enabled = checked
    }
  }

  Text {
    Layout.columnSpan: 4
    id: bounceCountStr
    color: "dimgrey"
    text: qsTr("BounceCount")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: bounceCount
    value: GlobalIlluminationCiVct.bounceCount
    minimumValue: 0
    maximumValue: 16
    decimals: 1
    onValueChanged: {
      GlobalIlluminationCiVct.bounceCount = value
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: highQuality
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("High Quality")
    checked: GlobalIlluminationCiVct.highQuality
    onToggled: {
      GlobalIlluminationCiVct.highQuality = checked;
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: anisotropic
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Anisotropic")
    checked: GlobalIlluminationCiVct.anisotropic
    onToggled: {
      GlobalIlluminationCiVct.anisotropic = checked;
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
    currentIndex: GlobalIlluminationCiVct.debugVisualizationMode
    model: ["Albedo", "Normal", "Emissive", "Lighting", "None"]
    onCurrentIndexChanged: {
      if (currentIndex < 0|| currentIndex > 4) {
        return;
      }
      GlobalIlluminationCiVct.debugVisualizationMode = currentIndex
    }
  }

  RoundButton {
    Layout.columnSpan: 1
    text: "\u21bb"
    Material.background: Material.primary
    onClicked: {
      combo.currentIndex = 0
      GlobalIlluminationCiVct.OnRefreshCameras();
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Refresh list all available cameras")
  }

  ComboBox {
    Layout.columnSpan: 5
    id: combo
    Layout.fillWidth: true
    model: GlobalIlluminationCiVct.cameraList
    currentIndex: 0
    onCurrentIndexChanged: {
      if (currentIndex < 0)
        return;
      GlobalIlluminationCiVct.OnCamareBind(textAt(currentIndex));
    }
    ToolTip.visible: hovered
    ToolTip.delay: tooltipDelay
    ToolTip.timeout: tooltipTimeout
    ToolTip.text: qsTr("Camera around which all cascades are centered from")
  }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}D{i:1}D{i:2}D{i:3}D{i:4}D{i:5}D{i:6}D{i:7}
D{i:8}D{i:9}D{i:10}D{i:11}D{i:12}D{i:13}D{i:14}D{i:15}D{i:16}D{i:17}D{i:18}D{i:19}
}
##^##*/
