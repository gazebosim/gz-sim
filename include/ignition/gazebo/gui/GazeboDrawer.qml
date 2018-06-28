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

  /**
   * Callback for list items
   */
  function onAction(_action) {
    switch(_action) {
      // Handle custom actions
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
    ListElement {
      title: "Load world"
      action: "loadWorld"
    }

    // Actions provided by Ignition GUI, with custom titles
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
      Material.theme: Material.theme
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
    title: "Load Gazebo world"
    folder: shortcuts.home
    nameFilters: [ "World files (*.world)" ]
    selectMultiple: false
    selectExisting: true
    onAccepted: {
      TmpIface.OnLoadWorld(fileUrl)
    }
  }
}
