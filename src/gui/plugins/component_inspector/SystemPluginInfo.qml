/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: systemInfoComponent
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Light text color according to theme
  property string propertyColor: Material.theme == Material.Light ? "#444444" : "#bbbbbb"

  // Dark text color according to theme
  property string valueColor: Material.theme == Material.Light ? "black" : "white"

  Column {
    anchors.fill: parent

    ExpandingTypeHeader {
      id: header
      customToolTip: "Information about system plugins attached to this entity"
    }

    // Content
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? column.height : 0
      clip: true
      color: "transparent"

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      ListView {
        id: column
        height: contentHeight
        model: componentData
        width: content.width
        spacing: 5

        delegate: Column {
          width: content.width
          height: pluginHeader.height + pluginContent.height

          Rectangle {
            id: pluginHeader
            width: content.width
            color: "transparent"
            height: 20
            RowLayout {
              anchors.fill: parent
              Item {
                width: margin * 2 + indentation
              }
              Image {
                id: icon
                sourceSize.height: indentation
                sourceSize.width: indentation
                fillMode: Image.Pad
                Layout.alignment : Qt.AlignVCenter
                source: pluginContent.show ?
                    "qrc:/Gazebo/images/minus.png" : "qrc:/Gazebo/images/plus.png"
              }
              Text {
                height: parent.height
                text: modelData[0].substring(modelData[0].lastIndexOf('::') + 2)
                Layout.alignment : Qt.AlignVCenter
                leftPadding: margin
                color: propertyColor
                font.pointSize: 12
              }
              Item {
                Layout.fillWidth: true
              }
            }
            MouseArea {
              anchors.fill: pluginHeader
              hoverEnabled: true
              cursorShape: Qt.PointingHandCursor
              onClicked: {
                pluginContent.show = !pluginContent.show
              }
              onEntered: {
                pluginHeader.color = highlightColor
              }
              onExited: {
                pluginHeader.color = "transparent"
              }
            }
          }
          GridLayout {
            id: pluginContent
            property bool show: false
            width: parent.width
            height: show ? 20 * 4 + innerXmlText.height : 0
            clip: true
            columns: 4

            Behavior on height {
              NumberAnimation {
                duration: 200;
                easing.type: Easing.InOutQuad
              }
            }
            Item {
              Layout.rowSpan: 4
              width: margin * 4 + indentation * 2
            }
            Text {
              height: 20
              text: "Name"
              color: propertyColor
              font.pointSize: 12
            }
            Text {
              height: 20
              text: modelData[0]
              color: valueColor
              font.pointSize: 12
              clip: true
              elide: Text.ElideLeft
              horizontalAlignment: Text.AlignRight
              Layout.fillWidth: true

              ToolTip {
                visible: maName.containsMouse
                delay: Qt.styleHints.mousePressAndHoldInterval
                text: modelData[0]
                enter: null
                exit: null
              }
              MouseArea {
                id: maName
                anchors.fill: parent
                hoverEnabled: true
              }
            }
            Item {
              Layout.rowSpan: 4
              width: margin
            }
            Text {
              height: 20
              text: "Filename"
              color: propertyColor
              font.pointSize: 12
            }
            Text {
              height: 20
              text: modelData[1]
              color: valueColor
              font.pointSize: 12
              clip: true
              elide: Text.ElideLeft
              horizontalAlignment: Text.AlignRight
              Layout.fillWidth: true

              ToolTip {
                visible: maFilename.containsMouse
                delay: Qt.styleHints.mousePressAndHoldInterval
                text: modelData[1]
                enter: null
                exit: null
              }
              MouseArea {
                id: maFilename
                anchors.fill: parent
                hoverEnabled: true
              }
            }
            Text {
              height: 20
              text: "Inner XML"
              color: propertyColor
              font.pointSize: 12
              Layout.columnSpan: 2
            }
            Rectangle {
              id: innerXmlRectangle
              Layout.columnSpan: 2
              Layout.fillWidth: true
              height: innerXmlText.paintedHeight
              color: Material.background
              TextEdit {
                id: innerXmlText
                width: parent.width
                text: modelData[2].length == 0 ? "N/A" : modelData[2].trim()
                color: valueColor
                textFormat: TextEdit.PlainText
                font.pointSize: 10
                clip: true
                readOnly: true
                wrapMode: Text.WordWrap
                selectByMouse: true
              }
            }
          }
        }
      }
    }
  }
}
