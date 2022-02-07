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
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  id: shapes
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  background: Rectangle {
    color: "transparent"
  }

  RowLayout {
    spacing: 2
    ToolButton {
      id: box
      ToolTip.text: "Point"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "pointlight.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        Lights.OnNewLightClicked("point")
      }
    }
    ToolButton{
      id: sphere
      ToolTip.text: "Directional"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "directionallight.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        Lights.OnNewLightClicked("directional")
      }
    }
    ToolButton {
      id: cylinder
      ToolTip.text: "Spot"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "spotlight.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        Lights.OnNewLightClicked("spot")
      }
    }
  }
}
