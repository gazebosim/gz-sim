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

Rectangle {
  id: typeHeader
  height: headerText.height
  width: headerText.width
  color: "transparent"

  // Custom tooltip text. If not set, the component's type name and ID will be
  // shown.
  property string customToolTip: ""

  function tooltipText(_model) {
    if (model === null)
      return "Unknown component"

    var info = '';
    if (model.typeName !== undefined)
      info += "Type name: " + model.typeName + "\n"

    if (model.typeId !== undefined)
      info += "Type Id: " + model.typeId + ""

    return info
  }

  Text {
    id: headerText
    text: model && model.shortName ? model.shortName : ''
    color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
    font.pointSize: 12

    ToolTip {
      visible: ma.containsMouse
      delay: tooltipDelay
      text: customToolTip.length > 0 ? customToolTip : typeHeader.tooltipText(componentData)
      y: typeHeader.y - 30
      enter: null
      exit: null
    }
    MouseArea {
      id: ma
      anchors.fill: parent
      hoverEnabled: true
      acceptedButtons: Qt.RightButton
    }
  }
}
