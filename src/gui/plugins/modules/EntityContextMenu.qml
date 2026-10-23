/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
import QtCore
import QtQuick 2.9
import QtQuick.Controls
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs
import QtQuick.Layouts 1.3
import GzSim 1.0 as GzSim

Item {
  Menu {
    id: menu
    transformOrigin: Menu.TopRight
    MenuItem {
      id: moveToMenu
      text: "Move To"
      onTriggered: context.OnRequest("move_to", context.entity)
    }
    MenuItem {
      id: followOptionsSubmenu
      text: "Follow Options >"
      MouseArea {
        id: followOptionsSubMouseArea
        anchors.fill: parent
        hoverEnabled: true
        onEntered: secondMenu.open()
      }
    }
    MenuItem {
      id: trackMenu
      text: "Track"
      onTriggered: context.OnRequest("track", context.entity)
    }
    MenuItem {
      id: removeMenu
      text: "Remove"
      onTriggered: context.OnRemove(context.entity, context.type)
    }
    MenuItem {
      id: copyMenu
      text: "Copy"
      onTriggered: context.OnRequest("copy", context.entity)
    }
    MenuItem {
      id: pasteMenu
      text: "Paste"
      // no data needs to be passed along to the paste request, so an empty
      // string is sent
      onTriggered: context.OnRequest("paste", "")
    }
    //   // cascading submenu only works in Qt 5.10+ on focal
    //   Menu {
    //     id: viewSubmenu
    //     title: "View"
    //     MenuItem {
    //       id: viewCollisionsMenu
    //       text: "Collisions"
    //       onTriggered: context.OnRequest("view_collisions", context.entity)
    //     }
    //   }
    // workaround for getting submenu's to work on bionic (< Qt 5.10)
    MenuItem {
      id: viewSubmenu
      text: "View >"
      MouseArea {
        id: viewSubMouseArea
        anchors.fill: parent
        hoverEnabled: true
        onEntered: thirdMenu.open()
      }
    }
    MenuItem {
      id: attachPlugin
      text: "Attach Plugin"
      onTriggered: {
        menu.close()
        pluginDialog.open()
      }
    }
  }
  Menu {
    id: secondMenu
    x: menu.x + menu.width
    y: menu.y + followOptionsSubmenu.y
    MenuItem {
      id: followMenu
      text: "Follow"
      onTriggered: {
        menu.close()
        context.OnRequest("follow", context.entity)
      }
    }
    MenuItem {
      id: followFreeLookMenu
      text: "Free Look"
      onTriggered: {
        menu.close()
        context.OnRequest("free_look", context.entity)
      }
    }
    MenuItem {
      id: followLookAtMenu
      text: "Look At"
      onTriggered: {
        menu.close()
        context.OnRequest("look_at", context.entity)
      }
    }
  }
  Menu {
    id: thirdMenu
    x: menu.x + menu.width
    y: menu.y + viewSubmenu.y
    MenuItem {
      id: viewCOMMenu
      text: "Center of Mass"
      onTriggered: {
        menu.close()
        context.OnRequest("view_com", context.entity)
      }
    }
    MenuItem {
      id: viewCollisionsMenu
      text: "Collisions"
      onTriggered: {
        menu.close()
        context.OnRequest("view_collisions", context.entity)
      }
    }
    MenuItem {
      id: viewInertiaMenu
      text: "Inertia"
      onTriggered: {
        menu.close()
        context.OnRequest("view_inertia", context.entity)
      }
    }
    MenuItem {
      id: viewJointsMenu
      text: "Joints"
      onTriggered: {
        menu.close()
        context.OnRequest("view_joints", context.entity)
      }
    }
    MenuItem {
      id: viewFramesMenu
      text: "Frames"
      onTriggered: {
        menu.close()
        context.OnRequest("view_frames", context.entity)
      }
    }
    MenuItem {
      id: viewTransparentMenu
      text: "Transparent"
      onTriggered: {
        menu.close()
        context.OnRequest("view_transparent", context.entity)
      }
    }
    MenuItem {
      id: viewWireframesMenu
      text: "Wireframe"
      onTriggered: {
        menu.close()
        context.OnRequest("view_wireframes", context.entity)
      }
    }
  }

  function open(_entity, _type, _eid) {
    context.entity = _entity
    context.type = _type
    context.eid = _eid
    moveToMenu.enabled = false
    followMenu.enabled = false
    followFreeLookMenu.enabled = false
    followLookAtMenu.enabled = false
    trackMenu.enabled = false
    removeMenu.enabled = false
    viewTransparentMenu.enabled = false;
    viewCOMMenu.enabled = false;
    viewInertiaMenu.enabled = false;
    viewJointsMenu.enabled = false;
    viewWireframesMenu.enabled = false;
    viewCollisionsMenu.enabled = false;
    viewFramesMenu.enabled = false;

    // enable / disable menu items
    if (context.type == "model" || context.type == "link" ||
        context.type == "visual" || context.type == "light" ||
        context.type == "performer")
    {
      moveToMenu.enabled = true
      followMenu.enabled = true
      followFreeLookMenu.enabled = true
      if (context.followingTarget)
      {
        followLookAtMenu.enabled = true
      }
      trackMenu.enabled = true
    }

    if (context.type == "model" || context.type == "light")
    {
      removeMenu.enabled = true
    }

    if (context.type == "model" || context.type == "link")
    {
      viewTransparentMenu.enabled = true;
      viewCOMMenu.enabled = true;
      viewInertiaMenu.enabled = true;
      viewWireframesMenu.enabled = true;
      viewCollisionsMenu.enabled = true;
    }

    if (context.type == "model")
    {
      viewJointsMenu.enabled = true;
    }

    // TODO(chapulina) Support collision, sensor, etc.
    if (context.type == "model" || context.type == "link" ||
        context.type == "visual")
    {
      viewFramesMenu.enabled = true
    }

    menu.popup()
  }

  Dialog {
    id: pluginDialog
    title: "Add Plugin Dialog"
    modal: true
    standardButtons: Dialog.Ok | Dialog.Cancel // Adds standard buttons
    parent: Overlay.overlay
    //parent: customPlugin.Window.window ? customPlugin.Window.window.contentItem : customPlugin
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    width: parent.width * 0.3
    height: parent.height * 0.6

    contentItem: Item {
      // Your dialog content goes here
      width: parent.width
      height: parent.height
      Column {
        anchors.centerIn: parent
        spacing: 10
        width: parent.width * 0.8 // Make inputs a bit narrower than parent
        height: parent.height

        Text {
          id: pluginNameLbl
          text: "Plugin Name:"
        }

        TextInput {
          id: pluginNames
          width: parent.width
          height: 20
          
        }

        Text {
          id: pluginFilesLbl
          text: "Plugin Filename:"
        }

        TextInput {
          id: pluginFilename
          width: parent.width
          height: 20
        }

        Text {
          id: pluginSdfLbl
          text: "Enter the inner-xml of your plugin"
        }

        TextArea {
            id: pluginParams
            width: parent.width
            height: parent.y + parent.height - (pluginSdfLbl.y + pluginFilesLbl.height)  // Set a reasonable height

            font.family: "Monospace"
            background: Rectangle {
                 color: "transparent" // No fill
                 border.width: 2     // No border
                 border.color: "lightblue"
            }
            leftPadding: 5
            topPadding: 5
            rightPadding: 5
            bottomPadding: 5
        }
      }
    }

    onAccepted: {
        console.log("Dialog Accepted!");
        context.OnAddPlugin(pluginNames.text, pluginFilename.text, pluginParams.text, context.eid)
        // Perform actions when OK is clicked
    }

    onRejected: {
        console.log("Dialog Rejected!");
        // Perform actions when Cancel is clicked or dialog is closed
    }
  }

  GzSim.EntityContextMenuItem {
    id: context
    property string entity
    property string type
    property int eid
  }
}
