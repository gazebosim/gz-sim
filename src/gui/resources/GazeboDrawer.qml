/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
import QtQuick.Dialogs 1.0

/**
 * Custom drawer
 */
Rectangle {
  id: customDrawer
  anchors.fill: parent
  color: Material.background

  property string saveWorldName: "";

  /**
   * Callback for list items
   */
  function onAction(_action) {
    switch(_action) {
      // Handle custom actions
      case "newWorld":
        TmpIface.OnNewWorld();
        break
      case "saveWorld":
        if (saveWorldName.length !== 0)
          TmpIface.OnSaveWorldAs(saveWorldName)
        else
          saveWorldDialog.open();
        break
      case "saveWorldAs":
        saveWorldDialog.open();
        break
      case "loadWorld":
        loadWorldDialog.open();
        break
      // Forward others to default drawer
      default:
        parent.onAction(_action);
        break
    }
  }

  ListModel {
    id: drawerModel

    // Custom action which calls custom C++ code
    /*ListElement {
      title: "New world"
      action: "newWorld"
      type: "world"
    }
    ListElement {
      title: "Load world"
      action: "loadWorld"
      type: "world"
    }
    ListElement {
      title: "Save world"
      action: "saveWorld"
      enabled: false
      type: "world"
    }
    ListElement {
      title: "Save world as..."
      action: "saveWorldAs"
      type: "world"
    }*/

    // Actions provided by Ignition GUI, with custom titles
    ListElement {
      title: "Load client configuration"
      action: "loadConfig"
    }
    ListElement {
      title: "Save client configuration"
      action: "saveConfig"
    }
    ListElement {
      title: "Save client configuration as"
      action: "saveConfigAs"
    }
    ListElement {
      title: "Style settings"
      action: "styleSettings"
    }
    ListElement {
      title: "Quit"
      action: "close"
    }
  }

  ListView {
    id: listView
    anchors.fill: parent

    delegate: ItemDelegate {
      width: parent.width
      text: title
      highlighted: ListView.isCurrentItem
      onClicked: {
        customDrawer.onAction(action);
        customDrawer.parent.closeDrawer();
      }
    }

    model: drawerModel

    ScrollIndicator.vertical: ScrollIndicator { }
  }

  /**
   * Load world dialog
   */
  FileDialog {
    id: loadWorldDialog
    title: "Load world"
    folder: shortcuts.home
    nameFilters: [ "World files (*.world, *.sdf)" ]
    selectMultiple: false
    selectExisting: true
    onAccepted: {
      TmpIface.OnLoadWorld(fileUrl)
    }
  }

  /**
   * Save world dialog
   */
  FileDialog {
    id: saveWorldDialog
    title: "Save world"
    folder: shortcuts.home
    nameFilters: [ "World files (*.world)" ]
    selectMultiple: false
    selectExisting: false
    onAccepted: {
      saveWorldName = fileUrl;
      TmpIface.OnSaveWorldAs(fileUrl);
    }
  }
}
