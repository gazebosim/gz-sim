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

// Item displaying spherical coordinates information.
Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Send new data to C++
  function sendCameraUpdate() {
    componentInspector.onCameraUpdate(
      horizontalFovSpin.value,
      imageWidthSpin.value,
      imageHeightSpin.value,
      nearClipSpin.value,
      farClipSpin.value
    );
  }

  Column {
    anchors.fill: parent

    // Header
    Rectangle {
      id: header
      width: parent.width
      height: typeHeader.height
      color: "transparent"

      RowLayout {
        anchors.fill: parent
        Item {
          width: margin
        }
        Image {
          id: icon
          sourceSize.height: indentation
          sourceSize.width: indentation
          fillMode: Image.Pad
          Layout.alignment : Qt.AlignVCenter
          source: content.show ?
              "qrc:/Gazebo/images/minus.png" : "qrc:/Gazebo/images/plus.png"
        }
        TypeHeader {
          id: typeHeader
        }
        Item {
          Layout.fillWidth: true
        }
      }
      MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: {
          content.show = !content.show
        }
        onEntered: {
          header.color = highlightColor
        }
        onExited: {
          header.color = "transparent"
        }
      }
    }

    // Content
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? grid.height : 0
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
        columns: 2

        // Horizontal field of view
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: horizontalFovText.width + indentation*3

          Text {
            id : horizontalFovText
            text: 'Horizontal FoV (°)'
            leftPadding: 4 
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
          ToolTip {
            visible: ma.containsMouse
            delay: Qt.styleHints.mousePressAndHoldInterval
            text: "Horizontal field of view (degrees)"
            enter: null
            exit: null
          }
          MouseArea {
            id: ma
            anchors.fill: content
            hoverEnabled: true
          }
        }
        IgnSpinBox {
          id: horizontalFovSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: model.data[0]
          value: horizontalFovSpin.activeFocus ? horizontalFovSpin.value : numberValue

          // This is equivalent to 0.1 radians, per the SDF spec
          minimumValue: 5.72958
          // This is equivalent to 6.283186 radians, per the SDF spec
          maximumValue: 360.0
          decimals:4 
          stepSize: 0.1
          onEditingFinished: {
            sendCameraUpdate()
          }
        }
        // End of Horizontal field of view


        // Image width 
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: imageWidthText.width + indentation*3

          Text {
            id : imageWidthText
            text: 'Image width (px)'
            leftPadding: 4 
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        IgnSpinBox {
          id: imageWidthSpin
          Layout.fillWidth: true
          property int numberValue: model.data[1]
          value: imageWidthSpin.activeFocus ? imageWidthSpin.value : numberValue

          minimumValue: 1
          maximumValue: 100000
          decimals:0 
          stepSize: 1
          onEditingFinished: {
            sendCameraUpdate()
          }
        }
        // End of Image width
        
        // Image height 
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: imageHeightText.width + indentation*3

          Text {
            id : imageHeightText
            text: 'Image height (px)'
            leftPadding: 4 
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        IgnSpinBox {
          id: imageHeightSpin
          Layout.fillWidth: true
          property int numberValue: model.data[2]
          value: imageHeightSpin.activeFocus ? imageHeightSpin.value : numberValue

          minimumValue: 1
          maximumValue: 100000
          decimals:0 
          stepSize: 1
          onEditingFinished: {
            sendCameraUpdate()
          }
        }
        // End of Image height
 
        // Near clip
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: nearClipText.width + indentation*3

          Text {
            id : nearClipText
            text: 'Near clip (m)'
            leftPadding: 4 
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        IgnSpinBox {
          id: nearClipSpin
          Layout.fillWidth: true
          property double numberValue: model.data[3]
          value: nearClipSpin.activeFocus ? nearClipSpin.value : numberValue

          minimumValue: 0.0 
          maximumValue: 100000
          decimals: 4 
          stepSize: 0.1 
          onEditingFinished: {
            sendCameraUpdate()
          }
        }
        // End of near clip
 
        // Far clip
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: farClipText.width + indentation*3

          Text {
            id : farClipText
            text: 'Far clip (m)'
            leftPadding: 4 
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        IgnSpinBox {
          id: farClipSpin
          Layout.fillWidth: true
          property double numberValue: model.data[4]
          value: farClipSpin.activeFocus ? farClipSpin.value : numberValue

          minimumValue: 0.0 
          maximumValue: 100000
          decimals: 4 
          stepSize: 0.1 
          onEditingFinished: {
            sendCameraUpdate()
          }
        }
        // End of far clip 
      }
    }
  }
}
