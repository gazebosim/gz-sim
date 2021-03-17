/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
  id: jointPositionController
  color: "transparent"
  Layout.minimumWidth: 400
  Layout.minimumHeight: 375
  anchors.fill: parent

  /**
   * Light grey according to theme
   */
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Dark grey according to theme
   */
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  /**
   * Forward posiiton changes to C++
   */
  function onCommand(_jointName, _pos) {
    JointPositionController.OnCommand(_jointName, _pos)
  }

  Connections {
    target: JointPositionController
    onLockedChanged: {
      lockButton.checked = JointPositionController.Locked()
    }
  }

  Rectangle {
    id: header
    height: lockButton.height
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    width: parent.width
    color: darkGrey

    RowLayout {
      anchors.fill: parent
      spacing: 0

      Label {
        text: JointPositionController.modelName
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 3
      }

      Item {
        height: entityLabel.height
        Layout.fillWidth: true
      }

      ToolButton {
        text: "?"
        font.bold: true
        ToolTip.text: "Help"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onClicked: {
          helpPopup.open();
        }
      }

      Popup {
        id: helpPopup
        parent: ApplicationWindow.overlay
        // From Qt 5.12 use
        // anchors.centerIn: parent
        x: 10
        y: 10
        Text {
          text: "If your model is not responding, make\n" +
                "sure all its joints have a \n" +
                "JointPositionController attached to\n" +
                "them, all using default topics."
        }
      }

      ToolButton {
        id: lockButton
        checkable: true
        checked: false
        text: "Lock entity"
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: lockButton.checked ? "qrc:/Gazebo/images/lock.svg" :
                                       "qrc:/Gazebo/images/unlock.svg"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        ToolTip.text: lockButton.checked ? "Unlock model selection"
            : "Lock model selection"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onToggled: {
          JointPositionController.locked = lockButton.checked
        }
      }

      ToolButton {
        id: resetButton
        text: "\u21BA"
        ToolTip.text: "Reset all joints to zero"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onClicked: {
          JointPositionController.OnReset()
        }
      }

      Label {
        id: entityLabel
        text: 'Entity ' + JointPositionController.modelEntity
        Layout.minimumWidth: 80
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 5
      }
    }
  }

  ListView {
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    clip: true
    ScrollBar.vertical: ScrollBar {
      policy: ScrollBar.AlwaysOn
    }
    model: {
      try {
        return JointsModel;
      }
      catch (e) {
        // The QML is loaded before we set the context property
        return null
      }
    }
    spacing: 0

    delegate: Loader {
      id: loader
      source: 'Joint.qml'
    }
  }
}
