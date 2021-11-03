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
import "qrc:/ComponentInspector"
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

  // Dummy function that a user of this component can overload 
  function onNoiseUpdate(_mean, _meanBias, _stdDev, _stdDevBias,
      _dynamicBiasStdDev, _dynamicBiasCorrelationTime) { }

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
              y: noise.y - 30
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
        IgnSpinBox {
          id: meanSpin
          Layout.fillWidth: true
          height: 40
          value: meanSpin.activeFocus ? meanSpin.value : meanValue 

          minimumValue: 0 
          maximumValue: 100000 
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            meanValue = meanSpin.value;
            onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
                dynamicBiasStdDev, dynamicBiasCorrelationTime);
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
              y: noise.y - 30
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
        IgnSpinBox {
          id: meanBiasSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: meanBias 
          value: meanBiasSpin.activeFocus ? meanBiasSpin.value : numberValue

          minimumValue: 0 
          maximumValue: 100000 
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            meanBias = meanBiasSpin.value;
            onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
                dynamicBiasStdDev, dynamicBiasCorrelationTime);
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
              y: noise.y - 30
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
        IgnSpinBox {
          id: stddevSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: stdDevValue
          value: stddevSpin.activeFocus ? stddevSpin.value : numberValue

          minimumValue: 0 
          maximumValue: 100000 
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            stdDevValue = stddevSpin.value;
            onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
                dynamicBiasStdDev, dynamicBiasCorrelationTime);
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
              y: noise.y - 30
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
        IgnSpinBox {
          id: stddevBiasSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: stdDevBias
          value: stddevBiasSpin.activeFocus ? stddevBiasSpin.value : numberValue

          minimumValue: 0 
          maximumValue: 100000 
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            stdDevBias = stddevBiasSpin.value;
            onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
                dynamicBiasStdDev, dynamicBiasCorrelationTime);
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
              y: noise.y - 30
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
        IgnSpinBox {
          id: dynamicBiasStdDevSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: dynamicBiasStdDev
          value: dynamicBiasStdDevSpin.activeFocus ? dynamicBiasStdDevSpin.value : numberValue

          minimumValue: 0 
          maximumValue: 100000 
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            dynamicBiasStdDev = dynamicBiasStdDevSpin.value;
            onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
                dynamicBiasStdDev, dynamicBiasCorrelationTime);
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
              y: noise.y - 30
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
        IgnSpinBox {
          id: dynamicBiasCorrelationTimeSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: dynamicBiasCorrelationTime
          value: dynamicBiasCorrelationTimeSpin.activeFocus ? dynamicBiasCorrelationTimeSpin.value : numberValue

          minimumValue: 0 
          maximumValue: 100000 
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            dynamicBiasCorrelationTime = dynamicBiasCorrelationTimeSpin.value;
            onNoiseUpdate(meanValue, meanBias, stdDevValue, stdDevBias,
                dynamicBiasStdDev, dynamicBiasCorrelationTime);
          }
        }
        // End of dynamic bias correlation time stddev bias
      }
    }
  }
}
