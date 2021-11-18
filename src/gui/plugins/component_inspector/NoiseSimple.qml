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

// Item displaying mean and std deviation noise information.
Rectangle {
  id: noiseSimple
  height: noiseContent.height
  color: "transparent"

  // Mean value
  property double meanValue: 0.0

  // Standard deviation value
  property double stdDevValue: 0.0

  // Signal that a user of this component can connect to in order to receive
  // noise updates 
  signal onNoiseUpdate(double _mean, double _stdDev)

  function onMean(_value) {
    meanValue = _value;
    onNoiseUpdate(meanValue, stdDevValue)
  }

  function onStdDev(_value) {
    stdDevValue = _value;
    onNoiseUpdate(meanValue, stdDevValue)
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

        // Mean
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: meanText.width

          Text {
            id : meanText
            text: 'Mean'
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
          minValue: -100000
          maxValue: 100000
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            meanSpin.onChange.connect(onMean)
          }
        }
        // End of mean
 
        // Std Dev
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: stddevText.width

          Text {
            id : stddevText
            text: 'Std. Deviation'
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
          maxValue: 100000
          stepValue: 0.1
          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            stddevSpin.onChange.connect(onStdDev)
          }
        }
        // End of stddev
      }
    }
  }
}
