/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying 3D vector information.
Rectangle {
  height: header.height + gzVectorInstance.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

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
          source: gzVectorInstance.expand ?
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
          gzVectorInstance.expand = !gzVectorInstance.expand
        }
        onEntered: {
          header.color = highlightColor
        }
        onExited: {
          header.color = "transparent"
        }
      }
    }
    Rectangle {
      color: "transparent"
      width: parent.width
      height: gzVectorInstance.height
      RowLayout {
        id: gzVectorRow
        width: parent.width

        // Left spacer
        Item {
          Layout.preferredWidth: margin + indentation
        }

        // Content
        GzVector3 {
          id: gzVectorInstance
          Layout.fillWidth: true
          Layout.preferredWidth: parent.width
          gzUnit: model && model.unit != undefined ? model.unit : 'm'

          xValue: model.data[0]
          yValue: model.data[1]
          zValue: model.data[2]

          // By default it is closed
          expand: false
        } // GzVector3 ends

        // Right spacer
        Item {
          Layout.preferredWidth: margin
        }
      } // end RowLayout
    } // end Rectangle
  } // Column ends
} // Rectangle ends
