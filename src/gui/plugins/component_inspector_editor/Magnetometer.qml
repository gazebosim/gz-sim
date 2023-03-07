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

// Item displaying magnetometerr noise information.
Rectangle {
  height: header.height + content.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  Column {
    anchors.fill: parent

    // The expanding header. Make sure that the content to expand has an id set
    // to the value "content".
    ExpandingTypeHeader {
      id: header

      // Set the 'expandingHeaderText' value to override the default header
      // values, which is based on the model.
      expandingHeaderText: "Magnetometer noise"
      expandingHeaderToolTip: "Magnetometer noise properties"
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

        // Space out the section header for x-axis noise.
        Rectangle {
          id: xaxisNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            id: xaxisNoiseText
            text: "X-axis Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the x-axis noise values.
        Noise {
          id: xaxisNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[0]
          meanBias: model.data[1]
          stdDevValue: model.data[2]
          stdDevBias: model.data[3]
          dynamicBiasStdDev: model.data[4]
          dynamicBiasCorrelationTime: model.data[5]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            xaxisNoise.onNoiseUpdate.connect(
                MagnetometerImpl.OnMagnetometerXNoise)
          }
        }

        // Space out the section header for y-axis noise.
        Rectangle {
          id: yaxisNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            id: yaxisNoiseText
            text: "Y-axis Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the y-axis noise values.
        Noise {
          id: yaxisNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[6]
          meanBias: model.data[7]
          stdDevValue: model.data[8]
          stdDevBias: model.data[9]
          dynamicBiasStdDev: model.data[10]
          dynamicBiasCorrelationTime: model.data[11]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            yaxisNoise.onNoiseUpdate.connect(
                MagnetometerImpl.OnMagnetometerYNoise)
          }
        }

        // Space out the section header for z-axis noise.
        Rectangle {
          id: zaxisNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            id: zaxisNoiseText
            text: "Z-axis Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the y-axis noise values.
        Noise {
          id: zaxisNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[12]
          meanBias: model.data[13]
          stdDevValue: model.data[14]
          stdDevBias: model.data[15]
          dynamicBiasStdDev: model.data[16]
          dynamicBiasCorrelationTime: model.data[17]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            zaxisNoise.onNoiseUpdate.connect(
                MagnetometerImpl.OnMagnetometerZNoise)
          }
        }
      }
    }
  }
}
