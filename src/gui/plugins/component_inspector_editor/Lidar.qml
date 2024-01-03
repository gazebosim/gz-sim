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

// Item displaying lidar sensor information.
Rectangle {
  height: header.height + content.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Noise mean value
  property double noiseMeanValue: model.data[0]

  // Noise mean bias
  property double noiseMeanBias: model.data[1]

  // Noise standard deviation value
  property double noieStdDevValue: model.data[2]

  // Noise standard deviation bias
  property double noiseStdDevBias: model.data[3]

  // Noise dynamic bias standard deviation
  property double noiseDynamicBiasStdDev: model.data[4]

  // Noise dynamic bias correlation time
  property double noiseDynamicBiasCorrelationTime: model.data[5]

  // Horizontal scan samples
  property double horizontalScanSamples: model.data[6]

  // Horizontal scan resolution
  property double horizontalScanResolution: model.data[7]

  // Horizontal scan min angle
  property double horizontalScanMinAngle: model.data[8]

  // Horizontal scan max angle
  property double horizontalScanMaxAngle: model.data[9]

  // Vertical scan samples
  property double verticalScanSamples: model.data[10]

  // Vertical scan resolution
  property double verticalScanResolution: model.data[11]

  // Vertical scan min angle
  property double verticalScanMinAngle: model.data[12]

  // Vertical scan max angle
  property double verticalScanMaxAngle: model.data[13]

  // Range min
  property double rangeMin: model.data[14]

  // Range max
  property double rangeMax: model.data[15]

  // Range resolution
  property double rangeResolution: model.data[16]

  // Callback when range min is changed
  function onRangeMin(_value) {
    rangeMin = _value
    LidarImpl.OnLidarChange(rangeMin, rangeMax, rangeResolution,
        horizontalScanSamples, horizontalScanResolution,
        horizontalScanMinAngle, horizontalScanMaxAngle,
        verticalScanSamples, verticalScanResolution,
        verticalScanMinAngle, verticalScanMaxAngle)
  }

  // Callback when range max is changed
  function onRangeMax(_value) {
    rangeMax = _value
    LidarImpl.OnLidarChange(rangeMin, rangeMax, rangeResolution,
        horizontalScanSamples, horizontalScanResolution,
        horizontalScanMinAngle, horizontalScanMaxAngle,
        verticalScanSamples, verticalScanResolution,
        verticalScanMinAngle, verticalScanMaxAngle)
  }

  // Callback when range resolution is changed
  function onRangeResolution(_value) {
    rangeResolution = _value
    LidarImpl.OnLidarChange(rangeMin, rangeMax, rangeResolution,
        horizontalScanSamples, horizontalScanResolution,
        horizontalScanMinAngle, horizontalScanMaxAngle,
        verticalScanSamples, verticalScanResolution,
        verticalScanMinAngle, verticalScanMaxAngle)
  }

  function onHorizontalScan(_samples, _resolution, _minAngle, _maxAngle) {
    horizontalScanSamples = _samples
    horizontalScanResolution = _resolution
    horizontalScanMinAngle = _minAngle
    horizontalScanMaxAngle = _maxAngle
    LidarImpl.OnLidarChange(rangeMin, rangeMax, rangeResolution,
        horizontalScanSamples, horizontalScanResolution,
        horizontalScanMinAngle, horizontalScanMaxAngle,
        verticalScanSamples, verticalScanResolution,
        verticalScanMinAngle, verticalScanMaxAngle)
  }

  function onVerticalScan(_samples, _resolution, _minAngle, _maxAngle) {
    verticalScanSamples = _samples
    verticalScanResolution = _resolution
    verticalScanMinAngle = _minAngle
    verticalScanMaxAngle = _maxAngle
    LidarImpl.OnLidarChange(rangeMin, rangeMax, rangeResolution,
        horizontalScanSamples, horizontalScanResolution,
        horizontalScanMinAngle, horizontalScanMaxAngle,
        verticalScanSamples, verticalScanResolution,
        verticalScanMinAngle, verticalScanMaxAngle)
  }

  Column {
    anchors.fill: parent

    // The expanding header. Make sure that the content to expand has an id set
    // to the value "content".
    ExpandingTypeHeader {
      id: header

      // Set the 'expandingHeaderText' value to override the default header
      // values, which is based on the model.
      expandingHeaderText: "Lidar"
      expandingHeaderToolTip: "Lidar properties"
    }

    // This is the content that will be expanded/contracted using the
    // ExpandingHeader above. Note the "id: content" attribute.
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? layout.height : 0
      clip: true
      color: "transparent"
      Layout.leftMargin: 4

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      ColumnLayout {
        id: layout
        width: parent.width

        // Space out the section header.
        Rectangle {
          id: rangeTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Range"
            color: "dimgrey"
            font.pointSize: 12
          }
        }
        // Range properties
        Rectangle {
          id: rangeRect
          width: parent.width
          Layout.fillWidth: true
          height: rangeGrid.height
          color: "transparent"
          Layout.leftMargin: 20

          GridLayout {
            id: rangeGrid
            width: parent.width
            columns: 4

            // Padding
            Rectangle {
              Layout.columnSpan: 4
              height: 4
            }

            // Range min
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: rangeMinText.width

              Text {
                id : rangeMinText
                text: 'Min (m)'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: minRangeMa.containsMouse
                  delay: tooltipDelay
                  text: "Minimum range distance in meters"
                  y: rangeMinText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: minRangeMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: rangeMinSpin
              Layout.fillWidth: true
              height: 40
              numberValue: rangeMin
              minValue: 0
              maxValue: Number.MAX_VALUE
              stepValue: 0.1
              // Connect to the onLidarUpdate signal in Noise.qml
              Component.onCompleted: {
                rangeMinSpin.onChange.connect(onRangeMin)
              }
            }
            // End of min range

            // Max range
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: rangeMaxText.width

              Text {
                id : rangeMaxText
                text: 'Max (m)'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: rangeMaxMa.containsMouse
                  delay: tooltipDelay
                  text: "Maximum range distance in meters"
                  y: rangeMaxText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: rangeMaxMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: rangeMaxSpin
              Layout.fillWidth: true
              height: 40
              numberValue: rangeMax
              minValue: 0
              maxValue: Number.MAX_VALUE
              stepValue: 0.1
              // Connect to the onNoiseUpdate signal in Noise.qml
              Component.onCompleted: {
                rangeMaxSpin.onChange.connect(onRangeMax)
              }
            }
            // End of range max

            // Range resolution
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: rangeResolutionText.width

              Text {
                id : rangeResolutionText
                text: 'Resolution'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: rangeResolutionMa.containsMouse
                  delay: tooltipDelay
                  text: "Range resolution"
                  y: rangeResolutionText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: rangeResolutionMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: rangeResolutionSpin
              Layout.fillWidth: true
              height: 40
              numberValue: rangeResolution
              minValue: 0
              maxValue: Number.MAX_VALUE
              stepValue: 0.1
              // Connect to the onLidarUpdate signal in Noise.qml
              Component.onCompleted: {
                rangeResolutionSpin.onChange.connect(onRangeResolution)
              }
            }
            // End of range resolution
          }
        }

        // Space out the section header.
        Rectangle {
          id: horizontalTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Horizontal scan"
            color: "dimgrey"
            font.pointSize: 12
          }
        }
        Rectangle {
          Layout.fillWidth: true
          height: horizontalScan.height
          Layout.leftMargin: 20
          color: "transparent"
          LidarScan {
            id: horizontalScan
            width: parent.width

            sampleValue: horizontalScanSamples
            resolutionValue: horizontalScanResolution
            minAngleValue: horizontalScanMinAngle
            maxAngleValue: horizontalScanMaxAngle

            // Connect to the onChange signal in Noise.qml
            Component.onCompleted: {
              horizontalScan.onChange.connect(onHorizontalScan)
            }
          }
        }

        // Space out the section header.
        Rectangle {
          id: verticalTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Vertical scan"
            color: "dimgrey"
            font.pointSize: 12
          }
        }
        Rectangle {
          Layout.fillWidth: true
          height: verticalScan.height
          Layout.leftMargin: 20
          color: "transparent"

          LidarScan {
            id: verticalScan
            width: parent.width

            sampleValue: verticalScanSamples
            resolutionValue: verticalScanResolution
            minAngleValue: verticalScanMinAngle
            maxAngleValue: verticalScanMaxAngle

            // Connect to the onChange signal in Noise.qml
            Component.onCompleted: {
              verticalScan.onChange.connect(onVerticalScan)
            }
          }
        }

        // Space out the section header.
        Rectangle {
          id: linearAccelerationXNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the noise values.
        Noise {
          id: noise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: noiseMeanValue
          meanBias: noiseMeanBias
          stdDevValue: noieStdDevValue
          stdDevBias: noiseStdDevBias
          dynamicBiasStdDev: noiseDynamicBiasStdDev
          dynamicBiasCorrelationTime: noiseDynamicBiasCorrelationTime

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            noise.onNoiseUpdate.connect(
                LidarImpl.OnLidarNoise)
          }
        }
      }
    }
  }
}
