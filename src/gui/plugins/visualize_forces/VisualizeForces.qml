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
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import IgnGazebo 1.0 as IgnGazebo

Rectangle {
  id: entityTree
  height: 200
  Layout.minimumWidth: 400
  Layout.minimumHeight: 320
  anchors.fill: parent

  /**
   * Color for even-numbered rows, according to current theme
   */
  property color even: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Color for odd-numbered rows, according to current theme
   */
  property color odd: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)


  ListModel {
      id: forceModel

      ListElement {
          link: ""
          plugin: "BuoyancyEngine"
          arrowColor: "#ff0000"
          isVisible: false
      }
      ListElement {
          link: "Buoyancy"
          plugin: 3.25
          arrowColor: "#0000ff"
          isVisible: false
      }
      ListElement {
          link: "Thruster"
          plugin: 1.95
          arrowColor: "#00ff00"
          isVisible: true
      }
  }

  Component {
    id: forceDelegate
    RowLayout {
      spacing: 0
      Label { 
        Layout.fillHeight: true
        Layout.fillWidth: true
        Layout.preferredHeight: implicitHeight
        Layout.preferredWidth: 100
        text: link
        background: Rectangle
        {
            color: (index % 2 == 0)? even : odd
        }
      }
      Label {
        Layout.fillHeight: true
        Layout.fillWidth: true
        Layout.preferredHeight: implicitHeight
        Layout.preferredWidth: 200
        text: plugin
         background: Rectangle
        {
            color: (index % 2 == 0)? even : odd
        }
      }
      
      Rectangle {
        width: 20
        height: 20
        color: arrowColor

         MouseArea {
          anchors.fill: parent
          hoverEnabled: true
          propagateComposedEvents: true
          onClicked: {
            mouse.accepted = true
            console.log("Color change not supported " + index)
          }
        }
      }
      CheckBox {
        checked: isVisible
        text: (isVisible) ? "visible" : "hidden"
        background: Rectangle
        {
          color: (index % 2 == 0)? even : odd
        }
      }
    }
  }

  ListView {
    id: tree
    anchors.fill: parent
    model: forceModel
    delegate: forceDelegate
  }
}