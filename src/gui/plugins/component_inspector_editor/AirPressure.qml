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

// Item displaying air pressure noise information.
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
      expandingHeaderText: "Air pressure"
      expandingHeaderToolTip: "Air pressure properties"
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

        RowLayout {
          id: rowLayout
          width: layout.width

          Text {
            leftPadding: 10
            id: referenceAltitudeText
            text: "Reference altitude (m)"
            color: "dimgrey"
            font.pointSize: 12
          }
          StateAwareSpin {
            id: referenceAltitudeSpin
            Layout.fillWidth: true
            height: 40
            numberValue: model.data[0]
            minValue: 0
            maxValue: Number.MAX_VALUE
            stepValue: 0.1
            // Connect to the onNoiseUpdate signal in Noise.qml
            Component.onCompleted: {
              referenceAltitudeSpin.onChange.connect(
                  AirPressureImpl.OnAirPressureReferenceAltitude)
            }
          }
        }

        // Space out the section header.
        Rectangle {
          id: noiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Pressure Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the pressure noise values.
        Noise {
          id: pressureNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[1]
          meanBias: model.data[2]
          stdDevValue: model.data[3]
          stdDevBias: model.data[4]
          dynamicBiasStdDev: model.data[5]
          dynamicBiasCorrelationTime: model.data[6]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            pressureNoise.onNoiseUpdate.connect(
                AirPressureImpl.OnAirPressureNoise)
          }
        }
      }
    }
  }
}
