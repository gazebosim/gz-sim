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
import QtQuick
import QtQuick.Controls
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtQuick.Controls.Styles

// Header
Rectangle {
  id: header
  width: parent.width
  height: typeHeader.height
  color: "transparent"

  // Horizontal margins
  property int margin: 5

  // Left indentation
  property int indentation: 10

  // Expose TypeHeader's custom tooltip
  property alias customToolTip: typeHeader.customToolTip

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
  ToolTip {
    visible: ma.containsMouse
    delay: tooltipDelay
    text: typeHeader.customToolTip.length > 0 ?
        typeHeader.customToolTip : typeHeader.tooltipText(componentData)
    y: typeHeader.y - 30
    enter: null
    exit: null
  }
  MouseArea {
    id: ma
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
