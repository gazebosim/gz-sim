/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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
import QtQuick.Dialogs 1.3
import GzSim 1.0 as GzSim

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

  property int selectedIndex: 0

  ColorDialog {
    id: colorDialog
    title: "Please choose a color"
    onAccepted: {
      ForceListModel.setColor(selectedIndex, colorDialog.color)
    }
    onRejected: {
    }
  }

  Component {
    id: forceDelegate
    RowLayout {
      spacing: 0
      Layout.alignment: Qt.AlignVCenter
      Label {
        Layout.fillHeight: true
        Layout.fillWidth: true
        Layout.preferredHeight: implicitHeight
        Layout.preferredWidth: 100
        text: link
        verticalAlignment: Text.AlignVCenter
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
        verticalAlignment: Text.AlignVCenter
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
            selectedIndex = index
            colorDialog.open()
          }
        }
      }
      CheckBox {
        checked: isVisible
        text: (isVisible) ? "visible" : "hidden"
        onClicked: function() {
          // Update model.
          ForceListModel.setVisibility(index, checked);
        }
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
    model: ForceListModel
    delegate: forceDelegate
  }
}