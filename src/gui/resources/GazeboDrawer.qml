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
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.3

/**
 * Custom drawer
 */
Rectangle {
  id: customDrawer
  anchors.fill: parent
  color: Material.background

  // Regex that matches a file:/// style absolute path, with a sdf extension
  property var fileValidator: /^file:\/\/(\/.+)+\.sdf$/;

  property bool lastSaveSuccess: false

  QtObject {
    id: sdfGenConfig
    property bool expandIncludeTags
    property bool saveFuelModelVersion
  }

  /**
   * Callback for list items
   */
  function onAction(_action) {
    switch(_action) {
      // Handle custom actions
      case "saveWorld":
        if (lastSaveSuccess)
          GuiFileHandler.SaveWorldAs(saveWorldFileText.text, sdfGenConfig)
        else
          sdfGenConfigDialog.open();
        break
      case "saveWorldAs":
        sdfGenConfigDialog.open();
        break
      case "aboutDialog":
        aboutDialog.open();
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
    ListElement {
      title: "Save world"
      actionElement: "saveWorld"
      enabled: false
      type: "world"
    }
    ListElement {
      title: "Save world as..."
      actionElement: "saveWorldAs"
      type: "world"
    }

    // Actions provided by Gazebo GUI, with custom titles
    ListElement {
      title: "Load client configuration"
      actionElement: "loadConfig"
    }
    ListElement {
      title: "Save client configuration"
      actionElement: "saveConfig"
    }
    ListElement {
      title: "Save client configuration as"
      actionElement: "saveConfigAs"
    }
    ListElement {
      title: "Style settings"
      actionElement: "styleSettings"
    }
    ListElement {
      title: "About"
      actionElement: "aboutDialog"
    }
    ListElement {
      title: "Quit"
      actionElement: "close"
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
        customDrawer.onAction(actionElement);
        customDrawer.parent.closeDrawer();
      }
    }

    model: drawerModel

    ScrollIndicator.vertical: ScrollIndicator { }
  }

  /**
   * Save world dialog
   */
  FileDialog {
    id: saveWorldDialog
    title: "Save world"
    folder: shortcuts.home
    nameFilters: [ "SDF files (*.sdf)" ]
    selectMultiple: false
    selectExisting: false
    onAccepted: {
      saveWorldFileText.text = fileUrl;
    }
  }

  /**
   * About dialog
   */
  Dialog {
    id: aboutDialog
    title: "Gazebo Sim"

    modal: true
    focus: true
    parent: ApplicationWindow.overlay
    width: parent.width / 3 > 500 ? 500 : parent.width / 3
    height: 300
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    closePolicy: Popup.CloseOnEscape
    standardButtons: StandardButton.Ok

    Text {
      anchors.fill: parent
      id: aboutMessage
      wrapMode: Text.Wrap
      verticalAlignment: Text.AlignVCenter
      color: Material.theme == Material.Light ? "black" : "white"
      textFormat: Text.RichText
      text: AboutDialogHandler.getVersionInformation()
      onLinkActivated: AboutDialogHandler.openURL(link)
    }
  }

  /**
   * Dialog with configurations for SDF generation
   */
  Dialog {
    id: sdfGenConfigDialog
    modal: true
    focus: true
    title: "File save options"
    parent: ApplicationWindow.overlay
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    closePolicy: Popup.CloseOnEscape
    onAccepted: {
      GuiFileHandler.SaveWorldAs(saveWorldFileText.text, sdfGenConfig);
    }
    Component.onCompleted: {
      dialogButtons.standardButton(Dialog.Ok).enabled = false
    }
    footer: DialogButtonBox {
      id: dialogButtons
      standardButtons: Dialog.Ok | Dialog.Cancel
    }
    contentItem: ColumnLayout {
      id: content

      RowLayout {
        Layout.fillWidth: true
        Label {
          text: "Location:"
        }
        TextField {
          id: saveWorldFileText
          text: "file:///"
          selectByMouse: true
          validator: RegExpValidator {
            regExp: fileValidator
          }
          onTextChanged: {
            var valid = saveWorldFileText.text.match(fileValidator)
            dialogButtons.standardButton(Dialog.Ok).enabled = valid
          }
        }
        Button {
          text: "Browse"
          onClicked: {
            saveWorldDialog.open()
          }
        }
      }

      CheckBox {
        text: "Expand include tags"
        Layout.fillWidth: true
        checked: sdfGenConfig.expandIncludeTags
        onClicked: {
          sdfGenConfig.expandIncludeTags = checked
        }
      }
      CheckBox {
        text: "Save Fuel model versions"
        Layout.fillWidth: true
        checked: sdfGenConfig.saveFuelModelVersion
        onClicked: {
          sdfGenConfig.saveFuelModelVersion = checked
        }
      }
    }
  }

  Connections {
    target: GuiFileHandler
    onNewSaveWorldStatus: {
      console.log(_msg);
      lastSaveSuccess = _status
      if (!_status) {
        fileSaveFailure.text =  _msg;
        fileSaveFailure.open();
      }
    }
  }

  /**
   * Message dialogs for failure messages emitted by GuiFileHandler
   */
  Dialog {
    id: fileSaveFailure
    property alias text: messageText.text
    title: "Error when saving world"

    modal: true
    focus: true
    parent: ApplicationWindow.overlay
    width: messageText.implicitWidth
    x: (parent.width - width) / 2
    y: (parent.height - height) / 2
    closePolicy: Popup.CloseOnEscape
    standardButtons: StandardButton.Cancel | StandardButton.Retry

    Label {
      anchors.fill: parent
      id: messageText
      wrapMode: Text.Wrap
      verticalAlignment: Text.AlignVCenter
    }

    onAccepted: {
      onAction("saveWorldAs")
    }
  }
}
