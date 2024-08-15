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

  background: Rectangle {
    color: "transparent"
  }

  RowLayout {
    spacing: 2
    ToolButton {
      id: box
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
        Shapes.OnMode("box")
      }
    }
    ToolButton{
      id: sphere
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
        Shapes.OnMode("sphere")
      }
    }
    ToolButton {
      id: cone
      ToolTip.text: "Cone"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "cone.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        Shapes.OnMode("cone")
      }
    }
    ToolButton {
      id: cylinder
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
        Shapes.OnMode("cylinder")
      }
    }
    ToolButton {
      id: capsule
      ToolTip.text: "Capsule"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "capsule.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        Shapes.OnMode("capsule")
      }
    }
    ToolButton {
      id: ellipsoid
      ToolTip.text: "Ellipsoid"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "ellipsoid.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        Shapes.OnMode("ellipsoid")
      }
    }
  }
}
