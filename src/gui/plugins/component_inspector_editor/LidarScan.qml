/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspectorEditor"
import "qrc:/qml"


// A component for displaying and editing a lidar scan.
Rectangle {
  width: parent.width
  height: grid.height
  color: "transparent"

  property int sampleValue: horizontalScanSamples
  property double resolutionValue: horizontalScanResolution
  property double minAngleValue: horizontalScanMinAngle
  property double maxAngleValue: horizontalScanMaxAngle

  // onChange signal
  signal onChange(int _samples, double _resolution, double _minAngle, double _maxAngle)

 function onSample(_value) {
   sampleValue = _value
   onChange(sampleValue, resolutionValue, minAngleValue, maxAngleValue)
 }

 function onResolution(_value) {
   resolutionValue = _value
   onChange(sampleValue, resolutionValue, minAngleValue, maxAngleValue)
 }

 function onMinAngle(_value) {
   minAngleValue = _value
   onChange(sampleValue, resolutionValue, minAngleValue, maxAngleValue)
 }

 function onMaxAngle(_value) {
   maxAngleValue = _value
   onChange(sampleValue, resolutionValue, minAngleValue, maxAngleValue)
 }

  GridLayout {
    id: grid
    width: parent.width
    columns: 4

    // Padding
    Rectangle {
      Layout.columnSpan: 4
      height: 4
    }

    // Samples
    Rectangle {
      color: "transparent"
      height: 40
      Layout.preferredWidth: sampleText.width

      Text {
        id : sampleText
        text: 'Samples'
        color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
        font.pointSize: 12
        anchors.centerIn: parent
        ToolTip {
          visible: sampleMa.containsMouse
          delay: tooltipDelay
          text: "Sample count"
          y: sampleText.y + 30
          enter: null
          exit: null
        }
        MouseArea {
          id: sampleMa
          anchors.fill: parent
          hoverEnabled: true
          acceptedButtons: Qt.RightButton
        }
      }
    }
    StateAwareSpin {
      id: sampleSpin
      Layout.fillWidth: true
      height: 40
      numberValue: sampleValue
      minValue: 0
      maxValue: Number.MAX_VALUE
      stepValue: 1
      // Send the change signal
      Component.onCompleted: {
        sampleSpin.onChange.connect(onSample)
      }
    }
    // End of samples

    // Resolution
    Rectangle {
      color: "transparent"
      height: 40
      Layout.preferredWidth: resolutionText.width

      Text {
        id : resolutionText
        text: 'Resolution'
        color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
        font.pointSize: 12
        anchors.centerIn: parent
        ToolTip {
          visible: resolutionMa.containsMouse
          delay: tooltipDelay
          text: "This number is multiplied by samples to determine the number of range data points returned."
          y: resolutionText.y + 30
          enter: null
          exit: null
        }
        MouseArea {
          id: resolutionMa
          anchors.fill: parent
          hoverEnabled: true
          acceptedButtons: Qt.RightButton
        }
      }
    }
    StateAwareSpin {
      id: resolutionSpin
      Layout.fillWidth: true
      height: 40
      numberValue: resolutionValue
      minValue: 0
      maxValue: Number.MAX_VALUE
      stepValue: 0.1
      // Send the change signal
      Component.onCompleted: {
        resolutionSpin.onChange.connect(onResolution)
      }
    }
    // End of resolution

    // Min angle
    Rectangle {
      color: "transparent"
      height: 40
      Layout.preferredWidth: minAngleText.width

      Text {
        id : minAngleText
        text: 'Min angle'
        color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
        font.pointSize: 12
        anchors.centerIn: parent
        ToolTip {
          visible: minAngleMa.containsMouse
          delay: tooltipDelay
          text: "Minimun angle"
          y: minAngleText.y + 30
          enter: null
          exit: null
        }
        MouseArea {
          id: minAngleMa
          anchors.fill: parent
          hoverEnabled: true
          acceptedButtons: Qt.RightButton
        }
      }
    }
    StateAwareSpin {
      id: minAngleSpin
      Layout.fillWidth: true
      height: 40
      numberValue: minAngleValue
      minValue: -3.1415
      maxValue: 3.1415
      stepValue: 0.1
      // Send the change signal
      Component.onCompleted: {
        minAngleSpin.onChange.connect(onMinAngle)
      }
    }
    // End of min angle

    // Max angle
    Rectangle {
      color: "transparent"
      height: 40
      Layout.preferredWidth: maxAngleText.width

      Text {
        id : maxAngleText
        text: 'Max angle'
        color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
        font.pointSize: 12
        anchors.centerIn: parent
        ToolTip {
          visible: maxAngleMa.containsMouse
          delay: tooltipDelay
          text: "Maximum angle"
          y: maxAngleText.y + 30
          enter: null
          exit: null
        }
        MouseArea {
          id: maxAngleMa
          anchors.fill: parent
          hoverEnabled: true
          acceptedButtons: Qt.RightButton
        }
      }
    }
    StateAwareSpin {
      id: maxAngleSpin
      Layout.fillWidth: true
      height: 40
      numberValue: maxAngleValue
      minValue: -3.1415
      maxValue: 3.1415
      stepValue: 0.1
      // Connect to the onLidarUpdate signal in Noise.qml
      Component.onCompleted: {
        maxAngleSpin.onChange.connect(onMaxAngle)
      }
    }
    // End of max angle
  }
}
