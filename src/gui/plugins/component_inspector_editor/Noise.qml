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

// Item displaying noise information.
Rectangle {
  id: noise
  height: noiseContent.height
  color: "transparent"

  // Mean value
  property double meanValue: 0.0

  // Mean bias
  property double meanBias: 0.0

  // Standard deviation value
  property double stdDevValue: 0.0

  // Standard deviation bias
  property double stdDevBias: 0.0

  // Dynamic bias standard deviation
  property double dynamicBiasStdDev: 0.0

  // Dynamic bias correlation time
  property double dynamicBiasCorrelationTime: 0.0

  // Signal that a user of this component can connect to in order to receive
  // noise updates
  signal onNoiseUpdate(double _mean, double _meanBias, double _stdDev,
      double _stdDevBias, double _dynamicBiasStdDev,
      double _dynamicBiasCorrelationTime)

  function onMean(_value) {
    meanValue = _value;
    onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
        dynamicBiasStdDev, dynamicBiasCorrelationTime);
  }

  function onMeanBias(_value) {
    meanBias = _value;
    onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
        dynamicBiasStdDev, dynamicBiasCorrelationTime);
  }

  function onStdDev(_value) {
    stdDevValue = _value;
    onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
        dynamicBiasStdDev, dynamicBiasCorrelationTime);
  }

  function onStdDevBias(_value) {
    stdDevBias = _value;
    onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
        dynamicBiasStdDev, dynamicBiasCorrelationTime);
  }

  function onDynamicBiasStdDev(_value) {
    dynamicBiasStdDev = _value;
    onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
        dynamicBiasStdDev, dynamicBiasCorrelationTime);
  }

  function onDynamicBiasCorrelationTime(_value) {
    dynamicBiasCorrelationTime = _value;
    onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
        dynamicBiasStdDev, dynamicBiasCorrelationTime);
  }

  // Display the main content
  Column {
    anchors.fill: parent

    // Content
    Rectangle {
      id: noiseContent
      width: parent.width
      height: grid.height
      clip: true
      color: "transparent"

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
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

        // Mean text label
        Text {
          Layout.columnSpan: 4
          text: "Mean"
          color: "dimgrey"
          font.pointSize: 10
          font.bold: true
        }

        // Mean
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: meanText.width

          Text {
            id : meanText
            text: 'Value'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
            ToolTip {
              visible: meanMa.containsMouse
              delay: tooltipDelay
              text: "Mean value"
              y: meanText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: meanMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }
          }
        }
        StateAwareSpin {
          id: meanSpin
          Layout.fillWidth: true
          height: 40
          numberValue: meanValue
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            meanSpin.onChange.connect(onMean)
          }
        }
        // End of mean

        // Mean Bias
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: meanBiasText.width

          Text {
            id : meanBiasText
            text: 'Bias'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
            ToolTip {
              visible: meanBiasMa.containsMouse
              delay: tooltipDelay
              text: "Mean bias"
              y: meanBiasText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: meanBiasMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }
          }
        }
        StateAwareSpin {
          id: meanBiasSpin
          Layout.fillWidth: true
          height: 40
          numberValue: meanBias
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            meanBiasSpin.onChange.connect(onMeanBias)
          }
        }
        // End of mean bias

        // Padding
        Rectangle {
          Layout.columnSpan: 4
          height: 4
        }

        // Std Dev text label
        Text {
          Layout.columnSpan: 4
          text: "Standard deviation"
          color: "dimgrey"
          font.pointSize: 10
          font.bold: true
        }

        // Std Dev
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: stddevText.width

          Text {
            id : stddevText
            text: 'Value'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
            ToolTip {
              visible: stdDevMa.containsMouse
              delay: tooltipDelay
              text: "Standard deviation value"
              y: stddevText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: stdDevMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }
          }
        }
        StateAwareSpin {
          id: stddevSpin
          Layout.fillWidth: true
          height: 40
          numberValue: stdDevValue
          minValue: 0
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            stddevSpin.onChange.connect(onStdDev)
          }
        }
        // End of stddev

        // Std Dev bias
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: stddevBiasText.width

          Text {
            id : stddevBiasText
            text: 'Bias'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
            ToolTip {
              visible: stddevBiasTextMa.containsMouse
              delay: tooltipDelay
              text: "Standard deviation bias"
              y: stddevBiasText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: stddevBiasTextMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }
          }
        }
        StateAwareSpin {
          id: stddevBiasSpin
          Layout.fillWidth: true
          height: 40
          numberValue: stdDevBias
          minValue: 0
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            stddevBiasSpin.onChange.connect(onStdDevBias)
          }
        }
        // End of stddev bias

        // Padding
        Rectangle {
          Layout.columnSpan: 4
          height: 4
        }

        // Dynamic bias text label
        Text {
          Layout.columnSpan: 4
          text: "Dynamic bias"
          color: "dimgrey"
          font.pointSize: 10
          font.bold: true
        }

        // Dynamic bias std Dev
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: dynamicBiasStddevText.width

          Text {
            id : dynamicBiasStddevText
            text: 'Std. Dev.'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
            ToolTip {
              visible: dynamicBiasStddevTextMa.containsMouse
              delay: tooltipDelay
              text: "Standard deviation"
              y: dynamicBiasStddevText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: dynamicBiasStddevTextMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }

          }
        }
        StateAwareSpin {
          id: dynamicBiasStdDevSpin
          Layout.fillWidth: true
          height: 40
          numberValue: dynamicBiasStdDev
          minValue: 0
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            dynamicBiasStdDevSpin.onChange.connect(onDynamicBiasStdDev)
          }
        }
        // End of dynamic bias stddev

        // Dynamic bias correlation time
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: dynamicBiasCorrelationTimeText.width

          Text {
            id : dynamicBiasCorrelationTimeText
            text: 'Cor. Time (s)'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent

            ToolTip {
              visible: dynamicBiasCorrelationTimeTextMa.containsMouse
              delay: tooltipDelay
              text: "Correlation time in seconds."
              y: dynamicBiasCorrelationTimeText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: dynamicBiasCorrelationTimeTextMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }
          }
        }
        StateAwareSpin {
          id: dynamicBiasCorrelationTimeSpin
          Layout.fillWidth: true
          height: 40
          numberValue: dynamicBiasCorrelationTime
          minValue: 0
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            dynamicBiasCorrelationTimeSpin.onChange.connect(onDynamicBiasCorrelationTime)
          }
        }
        // End of dynamic bias correlation time stddev bias
      }
    }
  }
}
