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
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

Rectangle {
  id: main
  property alias source: root.source
  property alias text: label.text
  property var worldData: null
  color: quickStart.isDark ?
      Material.color(Material.Grey, Material.Shade800) : "#ffffff"
  border.width: 2
  radius: 4
  clip: true

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 2
    spacing: 0

    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 26
      color: "transparent"

      Text {
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.leftMargin: 8
        anchors.rightMargin: 8
        id: label
        text: qsTr("Label")
        color: quickStart.textColor
        font.pixelSize: 13
        font.weight: Font.DemiBold
        renderType: Text.NativeRendering
        elide: Text.ElideRight
      }
    }

    Item {
      Layout.fillWidth: true
      Layout.fillHeight: true

      Image {
        id: root
        anchors.fill: parent
        source: ""
        fillMode: Image.PreserveAspectCrop
        asynchronous: true
      }
    }
  }

  MouseArea {
    anchors.fill: parent
    cursorShape: Qt.PointingHandCursor
    onClicked: quickStart.loadCatalogWorld(main.worldData)
    onDoubleClicked: quickStart.selectAndLaunchCatalogWorld(main.worldData)
  }
}
