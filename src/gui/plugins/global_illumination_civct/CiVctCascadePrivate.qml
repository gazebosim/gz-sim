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
  Layout.minimumHeight: 400
  Layout.columnSpan: 6
  Layout.fillWidth: true
  anchors.left: parent
  anchors.right: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  property var cascadePtr

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
    value: cascadePtr.resolutionX
    enabled: GlobalIlluminationCiVct.cascadesEditable
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      var tmpValue = value;
      tmpValue = nearestPowerOf2(tmpValue, oldValue);
      oldValue = tmpValue;
      value = tmpValue;
      cascadePtr.resolutionX = value;
    }
  }
  GzSpinBox {
    property int oldValue: -1

    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinY
    value: cascadePtr.resolutionY
    enabled: GlobalIlluminationCiVct.cascadesEditable
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      var tmpValue = value;
      tmpValue = nearestPowerOf2(tmpValue, oldValue);
      oldValue = tmpValue;
      value = tmpValue;
      cascadePtr.resolutionY = value;
    }
  }
  GzSpinBox {
    property int oldValue: -1

    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: resolutionSpinZ
    value: cascadePtr.resolutionZ
    enabled: GlobalIlluminationCiVct.cascadesEditable
    minimumValue: 4
    maximumValue: 512
    decimals: 1
    onEditingFinished: {
      var tmpValue = value;
      tmpValue = nearestPowerOf2(tmpValue, oldValue);
      oldValue = tmpValue;
      value = tmpValue;
      cascadePtr.resolutionZ = value;
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
    value: cascadePtr.octantCountX
    minimumValue: 1
    maximumValue: 8
    decimals: 1
    onEditingFinished: {
      cascadePtr.octantCountX = value
    }
  }
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountY
    value: cascadePtr.octantCountY
    minimumValue: 1
    maximumValue: 8
    decimals: 1
    onEditingFinished: {
      cascadePtr.octantCountY = value
    }
  }
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: octantCountZ
    value: cascadePtr.octantCountZ
    minimumValue: 1
    maximumValue: 8
    decimals: 1
    onEditingFinished: {
      cascadePtr.octantCountZ = value
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
    value: cascadePtr.thinWallCounter
    minimumValue: 0
    maximumValue: 5
    decimals: 2
    stepSize: 0.1
    onValueChanged: {
      cascadePtr.thinWallCounter = value
    }
  }

  Text {
    Layout.columnSpan: 6
    color: "dimgrey"
    text: qsTr("AreaHalfSize")
  }

  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: areaHalfSizeX
    value: cascadePtr.areaHalfSizeX
    enabled: GlobalIlluminationCiVct.cascadesEditable
    minimumValue: 0.01
    maximumValue: 100000
    decimals: 2
    onEditingFinished: {
      cascadePtr.areaHalfSizeX = value
    }
  }
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: areaHalfSizeY
    value: cascadePtr.areaHalfSizeY
    enabled: GlobalIlluminationCiVct.cascadesEditable
    minimumValue: 0.01
    maximumValue: 100000
    decimals: 2
    onEditingFinished: {
      cascadePtr.areaHalfSizeY = value
    }
  }
  GzSpinBox {
    Layout.columnSpan: 2
    Layout.fillWidth: true
    id: areaHalfSizeZ
    value: cascadePtr.areaHalfSizeZ
    enabled: GlobalIlluminationCiVct.cascadesEditable
    minimumValue: 0.01
    maximumValue: 100000
    decimals: 2
    onEditingFinished: {
      cascadePtr.areaHalfSizeZ = value
    }
  }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}D{i:1}D{i:2}D{i:3}D{i:4}D{i:5}D{i:6}D{i:7}
D{i:8}D{i:9}D{i:10}D{i:11}D{i:12}D{i:13}D{i:14}D{i:15}D{i:16}D{i:17}D{i:18}D{i:19}
}
##^##*/
