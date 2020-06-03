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
  id: insertmodel
  Layout.minimumWidth: 200
  Layout.minimumHeight: 500

  Component {
    id: localModel
    Item {
      width: 100
      height: 100
      Image {
        anchors.fill: parent
        source: "/home/john/.ignition/fuel/fuel.ignitionrobotics.org/openrobotics/models/Phone/1/thumbnails/1.png"
      }
      Text {
        text: "Phone"
      }
    }
  }
  ListView {
    anchors.fill: parent
    id: view
    ScrollBar.vertical: ScrollBar {}
    model: LocalModelList
    spacing: 10
    delegate:
    Rectangle {
      width: 300
      height: 30
      color: "#FF0000"
    }
  }
}
