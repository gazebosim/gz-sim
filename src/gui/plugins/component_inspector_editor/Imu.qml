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

// Item displaying imu sensor information.
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
      expandingHeaderText: "IMU"
      expandingHeaderToolTip: "Inertial measurement unit properties"
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
          id: linearAccelerationXNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Linear Acceleration X Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the linear acceleration X noise values.
        Noise {
          id: linearAccelerationXNoise
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
            linearAccelerationXNoise.onNoiseUpdate.connect(
                ImuImpl.OnLinearAccelerationXNoise)
          }
        }

        // Space out the section header.
        Rectangle {
          id: linearAccelerationYNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Linear Acceleration Y Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the linear acceleration Y noise values.
        Noise {
          id: linearAccelerationYNoise
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
            linearAccelerationYNoise.onNoiseUpdate.connect(
                ImuImpl.OnLinearAccelerationYNoise)
          }
        }

        // Space out the section header.
        Rectangle {
          id: linearAccelerationZNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Linear Acceleration Z Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the linear acceleration Z noise values.
        Noise {
          id: linearAccelerationZNoise
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
            linearAccelerationZNoise.onNoiseUpdate.connect(
                ImuImpl.OnLinearAccelerationZNoise)
          }
        }

        // Space out the section header.
        Rectangle {
          id: angularVelocityXNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Angular Velocity X Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the angular velocity X noise values.
        Noise {
          id: angularVelocityXNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[18]
          meanBias: model.data[19]
          stdDevValue: model.data[20]
          stdDevBias: model.data[21]
          dynamicBiasStdDev: model.data[22]
          dynamicBiasCorrelationTime: model.data[23]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            angularVelocityXNoise.onNoiseUpdate.connect(
                ImuImpl.OnAngularVelocityXNoise)
          }
        }

        // Space out the section header.
        Rectangle {
          id: angularVelocityYNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Angular Velocity Y Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the angular velocity Y noise values.
        Noise {
          id: angularVelocityYNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[18]
          meanBias: model.data[19]
          stdDevValue: model.data[20]
          stdDevBias: model.data[21]
          dynamicBiasStdDev: model.data[22]
          dynamicBiasCorrelationTime: model.data[23]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            angularVelocityYNoise.onNoiseUpdate.connect(
                ImuImpl.OnAngularVelocityYNoise)
          }
        }

        // Space out the section header.
        Rectangle {
          id: angularVelocityZNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Angular Velocity Z Noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the angular velocity Z noise values.
        Noise {
          id: angularVelocityZNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: model.data[18]
          meanBias: model.data[19]
          stdDevValue: model.data[20]
          stdDevBias: model.data[21]
          dynamicBiasStdDev: model.data[22]
          dynamicBiasCorrelationTime: model.data[23]

          // Connect to the onNoiseUpdate signal in Noise.qml
          Component.onCompleted: {
            angularVelocityZNoise.onNoiseUpdate.connect(
                ImuImpl.OnAngularVelocityZNoise)
          }
        }
      }
    }
  }
}
