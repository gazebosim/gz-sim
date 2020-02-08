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
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: booleanComponent
  height: typeHeader.height
  width: componentInspector.width
  color: "transparent"

  Row {
    spacing: booleanComponent.width - typeHeader.width - content.width - 20
    TypeHeader {
      id: typeHeader
    }

    Switch {
      id: content
      y: -content.width * 0.2
      checked: model.data
      enabled: false
    }
  }

  ToolTip {
    visible: ma.containsMouse
    delay: tooltipDelay
    text: content.checked ? "True" : "False"
    y: content.y - 30
    x: content.x
    enter: null
    exit: null
  }
  MouseArea {
    id: ma
    anchors.fill: booleanComponent
    hoverEnabled: true
    acceptedButtons: Qt.RightButton
  }
}
