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
import QtQuick.Layouts 1.3

Rectangle{
  id: main
  property alias source: root.source
  property alias text: label.text
  property string owner: ''
  border.color: "#e0e0e0"
  border.width: 2

  ColumnLayout {
    Rectangle {
      Text {
        x: 10
        y: 5
        id: label
        font.capitalization: Font.Capitalize
        text: qsTr("Label")
        color: "#443224"
        font.pixelSize: 14
      }
    }

    Rectangle {
      Image {
        id: root
        source: ""
        fillMode: Image.PreserveAspectCrop
        width: 216
        height: 124

        x: 2
        y: 24
        signal clicked

        MouseArea {
          anchors.fill: parent
          onClicked: quickStart.loadFuelWorld(main.text, main.owner);
        }
      }
    }
  }
}
