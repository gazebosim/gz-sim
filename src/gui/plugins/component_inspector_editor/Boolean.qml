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
import "qrc:/ComponentInspectorEditor"

Rectangle {
  id: booleanComponent
  height: typeHeader.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  RowLayout {
    anchors.fill: parent

    Item {
      height: parent.height
      width: margin
    }

    Item {
      height: parent.height
      width: indentation
    }

    TypeHeader {
      id: typeHeader
    }

    Rectangle {
      id: content
      Layout.fillWidth: true
      height: typeHeader.height
      color: "transparent"

      Switch {
        id: booleanSwitch
        anchors.right: content.right
        height: typeHeader.height * 0.8
        y: typeHeader.height * 0.1
        checked: model.data
        enabled: false

        Binding {
          target: booleanSwitch.indicator
          property: 'height'
          value: booleanSwitch.height
        }

        // groove
        Binding {
          target: (booleanSwitch.indicator ? booleanSwitch.indicator.children[0] : null)
          property: 'height'
          value: booleanSwitch.height * 0.8
        }

        // handle
        Binding {
          target: (booleanSwitch.indicator ? booleanSwitch.indicator.children[1] : null)
          property: 'height'
          value: booleanSwitch.height
        }
      }

      ToolTip {
        visible: ma.containsMouse
        delay: tooltipDelay
        text: content.checked ? "True" : "False"
        enter: null
        exit: null
      }
      MouseArea {
        id: ma
        anchors.fill: content
        hoverEnabled: true
        acceptedButtons: Qt.RightButton
      }
    }

    Item {
      height: parent.height
      width: margin
    }
  }
}
