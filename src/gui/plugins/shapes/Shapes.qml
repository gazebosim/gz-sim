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

  property color snapTitle: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade900) :
    Material.color(Material.Grey, Material.Shade200)

  property color snapItem: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade800) :
    Material.color(Material.Grey, Material.Shade100)

  function windowWidth() {
    return transformControl.Window.window ? (transformControl.Window.window.width) : 0
  }

  function windowHeight() {
    return transformControl.Window.window ? (transformControl.Window.window.height) : 0
  }

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  RowLayout {
    spacing: 2
    ToolButton {
      id: box
      checkable: true
      checked: true
      ButtonGroup.group: group
      ToolTip.text: "Box"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "box.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
      }
    }
    ToolButton{
      id: sphere
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Sphere"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "sphere.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
      }
    }
    ToolButton {
      id: cylinder
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Cylinder"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "cylinder.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
      }
    }
  }
}
