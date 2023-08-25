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
  columns: 6
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 800
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  function isPowerOf2(n) {
    return (n & n-1) === 0;
  }

  // Returns the closest power of 2, rounding down if need to.
  //  floorPowerOf2( 4 ) = 4
  //  floorPowerOf2( 5 ) = 4
  function floorPowerOf2(n) {
    return 1 << (31 - Math.clz32(n));
  }

  // Returns the closest power of 2, rounding up if need to.
  //  floorPowerOf2( 4 ) = 4
  //  floorPowerOf2( 5 ) = 8
  function ceilPowerOf2(n) {
      if (isPowerOf2(n)) {
        return n;
      }
    return 1 << (31 - Math.clz32(n) + 1);
  }

  function nearestPowerOf2(n, oldValue=undefined) {
    if (oldValue === undefined) {
      return floorPowerOf2(n);
    }
    else {
      if (oldValue <= n) {
        return ceilPowerOf2(n);
      }
      else {
        return floorPowerOf2(n);
      }
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: displayVisual
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Enabled")
    checked: GlobalIlluminationVct.enabled
    onToggled: {
      GlobalIlluminationVct.enabled = checked
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
    value: GlobalIlluminationVct.bounceCount
    minimumValue: 0
    maximumValue: 16
    decimals: 1
    onValueChanged: {
      GlobalIlluminationVct.bounceCount = value
    }
  }

  Text {
    Layout.columnSpan: 6
    id: resolutionStr
    color: "dimgrey"
    text: qsTr("Resolution")
  }

  GzSpinBox {
    property int oldValue: -1

    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinX
    value: GlobalIlluminationVct.resolutionX
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      var tmpValue = value;
      tmpValue = nearestPowerOf2(tmpValue, oldValue);
      oldValue = tmpValue;
      value = tmpValue;
      GlobalIlluminationVct.resolutionX = value;
    }
  }
  GzSpinBox {
    property int oldValue: -1

    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinY
    value: GlobalIlluminationVct.resolutionY
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      var tmpValue = value;
      tmpValue = nearestPowerOf2(tmpValue, oldValue);
      oldValue = tmpValue;
      value = tmpValue;
      GlobalIlluminationVct.resolutionY = value;
    }
  }
  GzSpinBox {
    property int oldValue: -1

    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinZ
    value: GlobalIlluminationVct.resolutionZ
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      var tmpValue = value;
      tmpValue = nearestPowerOf2(tmpValue, oldValue);
      oldValue = tmpValue;
      value = tmpValue;
      GlobalIlluminationVct.resolutionZ = value;
    }
  }

  Text {
    Layout.columnSpan: 6
    id: octantCountStr
    color: "dimgrey"
    text: qsTr("OctantCount")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountX
    value: GlobalIlluminationVct.octantCountX
    minimumValue: 1
    maximumValue: 8
    decimals: 1
    onEditingFinished: {
      GlobalIlluminationVct.octantCountX = value
    }
  }
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountY
    value: GlobalIlluminationVct.octantCountY
    minimumValue: 1
    maximumValue: 8
    decimals: 1
    onEditingFinished: {
      GlobalIlluminationVct.octantCountY = value
    }
  }
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountZ
    value: GlobalIlluminationVct.octantCountZ
    minimumValue: 1
    maximumValue: 8
    decimals: 1
    onEditingFinished: {
      GlobalIlluminationVct.octantCountZ = value
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: conserveMemory
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Conserve Memory")
    checked: GlobalIlluminationVct.conserveMemory
    onToggled: {
        GlobalIlluminationVct.conserveMemory = checked;
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: highQuality
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("High Quality")
    checked: GlobalIlluminationVct.highQuality
    onToggled: {
      GlobalIlluminationVct.highQuality = checked;
    }
  }

  CheckBox {
    Layout.alignment: Qt.AlignHCenter
    id: anisotropic
    Layout.columnSpan: 6
    Layout.fillWidth: true
    text: qsTr("Anisotropic")
    checked: GlobalIlluminationVct.anisotropic
    onToggled: {
      GlobalIlluminationVct.anisotropic = checked;
    }
  }

  Text {
    Layout.columnSpan: 4
    id: thinWallCounterStr
    color: "dimgrey"
    text: qsTr("ThinWallCounter")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: thinWallCounter
    value: GlobalIlluminationVct.thinWallCounter
    minimumValue: 0
    maximumValue: 5
    decimals: 2
    stepSize: 0.1
    onValueChanged: {
      GlobalIlluminationVct.thinWallCounter = value
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
    currentIndex: GlobalIlluminationVct.debugVisualizationMode
    model: ["Albedo", "Normal", "Emissive", "Lighting", "None"]
    onCurrentIndexChanged: {
      if (currentIndex < 0|| currentIndex > 4) {
        return;
      }
      GlobalIlluminationVct.debugVisualizationMode = currentIndex
    }
  }
}
