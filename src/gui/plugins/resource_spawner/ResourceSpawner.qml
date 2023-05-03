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
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

import QtQml.Models 2.2

// Wrap everything in a rectangle so that there are no transparent
// areas at the end of the trees
Rectangle {
  id: resourceSpawner
  color: Material.background
  Layout.minimumWidth: 740
  Layout.minimumHeight: 500
  anchors.fill: parent

  /**
   * Height of each item on the tree
   */
  property int treeItemHeight: 30;

  /**
   * Height of each item on the grid
   */
  property int gridItemHeight: 150;

  /**
   * Width of each item on the grid
   */
  property int gridItemWidth: 200;

  /**
   * Currently selected path
   */
  property string currentPath: "";

  /**
   * Color for odd rows, according to theme
   */
  property color oddColor: (Material.theme == Material.Light) ?
  Material.color(Material.Grey, Material.Shade100):
  Material.color(Material.Grey, Material.Shade800);

  /**
   * Color for even rows, according to theme
   */
  property color evenColor: (Material.theme == Material.Light) ?
  Material.color(Material.Grey, Material.Shade200):
  Material.color(Material.Grey, Material.Shade900);

  function windowWidth() {
    return resourceSpawner.Window.window ? (resourceSpawner.Window.window.width) : 0
  }

  function windowHeight() {
    return resourceSpawner.Window.window ? (resourceSpawner.Window.window.height) : 0
  }

  SplitView {
    anchors.fill: parent
    orientation: Qt.Horizontal

    SplitView {
      orientation: Qt.Vertical
      Layout.minimumHeight: 400
      Layout.minimumWidth: 315
      anchors.bottom: parent.bottom
      anchors.top: parent.top
      anchors.left: parent.left
      height: parent.height
      ColumnLayout {
        id: localColumn
        Layout.minimumHeight: 100
        Layout.fillWidth: true
        spacing: 0
        Rectangle {
          color: evenColor
          border.color: "gray"
          border.width: 1
          Layout.alignment: Qt.AlignLeft
          Layout.preferredHeight: 35
          Layout.fillWidth: true
          Layout.leftMargin: -border.width
          Layout.rightMargin: -border.width
          Label {
            padding: 5
            text: "Local resources"
            anchors.fill: parent
            font.pointSize: 14
          }
        }
        TreeView {
          id: treeView
          model: PathList
          Layout.fillWidth: true
          Layout.fillHeight: true
          Layout.alignment: Qt.AlignCenter
          Layout.minimumWidth: 300
          Layout.minimumHeight: 100
          // For some reason, SingleSelection is not working
          selectionMode: SelectionMode.MultiSelection
          verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
          headerVisible: false
          backgroundVisible: false
          frameVisible: false

          headerDelegate: Rectangle {
            visible: false
          }

          TableViewColumn
          {
            role: "name"
          }

          selection: ItemSelectionModel {
            model: PathList
          }

          style: TreeViewStyle {
            indentation: 0
            rowDelegate: Rectangle {
              id: row
              color: Material.background
              height: treeItemHeight
            }
            itemDelegate: Rectangle {
              id: localItem
              color: styleData.selected ? Material.accent : (styleData.row % 2 == 0) ? evenColor : oddColor
              height: treeItemHeight

              anchors.top: parent.top
              anchors.right: parent.right

              Image {
                id: dirIcon
                source: styleData.selected ? "folder_open.png" : "folder_closed.png"
                height: treeItemHeight * 0.6
                width: treeItemHeight * 0.6
                anchors.verticalCenter: parent.verticalCenter
                anchors.left: parent.left
              }

              Label {
                text: (model === null) ? "" : model.path
                elide: Text.ElideMiddle
                font.pointSize: 12
                anchors.leftMargin: 1
                anchors.left: dirIcon.right
                anchors.verticalCenter: parent.verticalCenter
                leftPadding: 2
              }

              MouseArea {
                id: ma
                anchors.fill: parent
                propagateComposedEvents: true
                hoverEnabled: true
                onClicked: {
                  ResourceSpawner.OnPathClicked(model.path);
                  ResourceSpawner.DisplayResources();
                  currentPath = model.path
                  gridView.currentIndex = -1
                  mouse.accepted = false
                  treeView.selection.select(styleData.index, ItemSelectionModel.ClearAndSelect)
                  treeView2.selection.clearSelection()
                }
              }

              ToolTip {
                visible: ma.containsMouse
                delay: 500
                y: localItem.z - 30
                text: model === null ?
                "?" : model.path
                enter: null
                exit: null
              }
            }
          }
        }
      }

      ColumnLayout {
        id: fuelColumn
        Layout.minimumHeight: 100
        spacing: 0
        Rectangle {
          color: evenColor
          border.color: "gray"
          Layout.alignment: Qt.AlignLeft
          Layout.preferredHeight: 35
          Layout.fillWidth: true
          border.width: 1
          Layout.leftMargin: -border.width
          Layout.rightMargin: -border.width
          Layout.topMargin: -border.width
          Label {
            text: "Fuel resources"
            padding: 5
            anchors.fill: parent
            font.pointSize: 14
          }
        }

        ListView {
          id: listView
          model: OwnerList
          Layout.fillWidth: true
          Layout.fillHeight: true
          Layout.minimumWidth: 300
          clip: true

          ScrollBar.vertical: ScrollBar {
            active: true;
          }

          delegate: Rectangle {
            id: fuelItem2
            color: ListView.view.currentIndex == index ? Material.accent : (index % 2 == 0) ? evenColor : oddColor
            height: treeItemHeight
            width: ListView.view.width
            ListView.onAdd : {
              ListView.view.currentIndex = index
            }

            ListView.onCurrentItemChanged: {
              if (index >= 0) {
                currentPath = model.path
                ResourceSpawner.OnOwnerClicked(model.path)
                ResourceSpawner.DisplayResources();
                treeView.selection.clearSelection()
                gridView.currentIndex = -1
              }
            }

            MouseArea {
              anchors.fill: parent
              onClicked: {
                listView.currentIndex = index
              }
            }

            RowLayout {
              anchors.fill: parent
              anchors.leftMargin: 10
              anchors.rightMargin: 10
              clip: true

              Image {
                id: dirIcon2
                source: listView.currentIndex == index ? "folder_open.png" : "folder_closed.png"
                Layout.preferredHeight: treeItemHeight * 0.6
                Layout.preferredWidth: treeItemHeight * 0.6
              }

              Label {
                text: model.path
                Layout.fillWidth: true
                elide: Text.ElideMiddle
                font.pointSize: 12
                leftPadding: 2
              }

              Button {
                // unicode for emdash (—)
                text: "\u2014"
                flat: true
                Layout.fillHeight : true
                Layout.preferredWidth: 30
                visible: !ResourceSpawner.IsDefaultOwner(model.path)

                onClicked: {
                  ResourceSpawner.RemoveOwner(model.path)
                }
              }
            }
          }
        }

        // Add owner button
        Rectangle {
          id: addOwnerBar
          color: evenColor
          Layout.minimumHeight: 50
          Layout.fillWidth: true
          clip:true
          RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            spacing: 10

            TextField {
              Layout.fillWidth: true
              id: ownerInput
              selectByMouse: true
              color: Material.theme == Material.Light ? "black" : "white"
              placeholderText: "Add owner"
              function processInput() {
                if (text != "" && ResourceSpawner.AddOwner(text)) {
                  text = ""
                }
              }
              onAccepted: {
                processInput();
              }
            }

            RoundButton {
              Material.background: Material.Green
              contentItem: Label {
                text: "+"
                color: "white"
                font.pointSize: 30
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
              }
              padding: 0
              onClicked: {
                ownerInput.processInput()
              }
            }
          }
        }
      }
    } // SplitView

    ColumnLayout {
      Layout.minimumWidth: 300
      Layout.fillWidth: true
      spacing: 0
      Rectangle {
        id: searchSortBar
        color: evenColor
        height: 50
        Layout.minimumWidth: 290
        Layout.minimumHeight: 50
        Layout.fillWidth: true
        RowLayout {
          id: rowLayout
          spacing: 7
          anchors.fill: parent
          Rectangle {
            color: "transparent"
            height: 25
            width: 25
            Layout.leftMargin: 15
            Image {
              id: searchIcon
              source: "qrc:/Gazebo/images/search.svg"
              anchors.verticalCenter: parent.verticalCenter
            }
          }
          Rectangle {
            color: oddColor
            height: 35
            Layout.minimumWidth: 100
            Layout.minimumHeight: 30
            Layout.preferredWidth: (searchSortBar.width - 80) / 2
            TextInput {
              id: searchField
              anchors.fill: parent
              topPadding: 8
              leftPadding: 5
              selectByMouse: true
              color: Material.theme == Material.Light ? "black" : "white"
              onTextEdited: {
                ResourceSpawner.OnSearchEntered(searchField.text);
                ResourceSpawner.DisplayResources();
              }
            }
          }
          Rectangle {
            color: "transparent"
            implicitHeight: sortComboBox.implicitHeight
            Layout.minimumWidth: 140
            Layout.preferredWidth: (searchSortBar.width - 80) / 2
            ComboBox {
              id: sortComboBox
              anchors.fill: parent
              model: ListModel {
                id: cbItems
                ListElement { text: "Most Recent"}
                ListElement { text: "A - Z"}
                ListElement { text: "Z - A"}
                ListElement { text: "Downloaded"}
              }
              onActivated: {
                ResourceSpawner.OnSortChosen(cbItems.get(currentIndex).text);
                ResourceSpawner.DisplayResources();
              }
            }
          }
        }
      }

      Rectangle {
        Layout.fillWidth: true
        Layout.minimumWidth: 300
        height: 40
        color: Material.accent
        Label {
          text: currentPath ? "Owner: " + currentPath + " (" + gridView.model.totalCount + ")" : ""
          font.pointSize: 12
          elide: Text.ElideMiddle
          anchors.margins: 5
          anchors.verticalCenter: parent.verticalCenter
          anchors.left: parent.left
          anchors.right: parent.right
        }
      }

      Rectangle {
        Layout.fillHeight: true
        Layout.fillWidth: true
        color: Material.background
        GridView {
          id: gridView
          model: ResourceList
          cellWidth: gridItemWidth
          cellHeight: gridItemHeight
          currentIndex: -1
          clip: true
          anchors.fill: parent
          anchors.margins: 5

          ScrollBar.vertical: ScrollBar {
            active: true;
          }

          delegate: Pane {
            id: itemDelegate
            width: gridView.cellWidth - 8
            height: gridView.cellHeight - 8
            Material.elevation: 6
            background: Rectangle {
              color: gridView.currentIndex == index ? Material.accent : Material.background
              layer.enabled: true
              layer.effect: ElevationEffect {
                elevation: 6
              }
              border.width: 1
              border.color: "lightgray"
            }

            ColumnLayout {
              anchors.fill: parent
              Label {
                text: model.name
                font.pointSize: 12
                elide: Text.ElideRight
                height: 40
                Layout.fillWidth: true
              }
              Image {
                id: thumbnailImage
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.margins: 1
                source: (model.isFuel && !model.isDownloaded) ?
                "DownloadToUse.png" :
                (model.thumbnail === "" ?
                "NoThumbnail.png" : "file:" + model.thumbnail)
                fillMode: Image.PreserveAspectFit
              }
            }
            MouseArea {
              id: ma2
              anchors.fill: parent
              hoverEnabled: true
              propagateComposedEvents: true
              onClicked: {
                if (model.isFuel && !model.isDownloaded)
                {
                  downloadDialog.open()
                }
                else
                {
                  ResourceSpawner.OnResourceSpawn(model.sdf);
                  gridView.currentIndex = index;
                }
              }
            }
            Dialog {
              id: downloadDialog
              parent: resourceSpawner.Window.window ? resourceSpawner.Window.window.contentItem : resourceSpawner
              x: (windowWidth() - width) / 2
              y: (windowHeight() - height) / 2
              width: 360
              height: 150
              modal: true
              focus: true
              title: "Note"
              standardButtons: Dialog.Ok
              Rectangle {
                color: "transparent"
                anchors.fill: parent
                Label {
                  width: downloadDialog.width - 50
                  height: downloadDialog.height
                  text: "Please download the resource first by clicking the cloud icon."
                  wrapMode: Text.WordWrap
                }
              }
            }
            ToolTip {
              visible: ma2.containsMouse
              delay: 500
              text: model === null ? "N/A" : model.name
              enter: null
              exit: null
            }
            ToolButton {
              height: 35
              width: 35
              checkable: false
              ToolTip.text: "Download"
              ToolTip.visible: hovered
              ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
              anchors.bottom: parent.bottom
              anchors.right: parent.right
              anchors.rightMargin: -10
              anchors.bottomMargin: -10
              visible: model.isFuel && !model.isDownloaded
              contentItem: Image {
                fillMode: Image.PreserveAspectFit
                horizontalAlignment: Image.AlignHCenter
                verticalAlignment: Image.AlignVCenter
                source: "CloudDownload.png"
                sourceSize.width: 35;
                sourceSize.height: 35;
              }
              onClicked: {
                ResourceSpawner.OnDownloadFuelResource(model.sdf, model.name, model.owner, model.index)
                model.isDownloaded = true
              }
            }
          }
        }
      }
    }
  }

  // Dialog for error messages
  Dialog {
    id: messageDialog
    width: 360
    height: 150
    parent: resourceSpawner.Window.window ? resourceSpawner.Window.window.contentItem : resourceSpawner
    x: Math.round((parent.width - width) / 2)
    y: Math.round((parent.height - height) / 2)
    modal: true
    focus: true
    title: "Error"
    standardButtons: Dialog.Ok
    contentItem: Text {
      text: ""
    }
  }

  Connections {
    target: ResourceSpawner
    onResourceSpawnerError : {
      messageDialog.contentItem.text = _errorMsg
      messageDialog.visible = true
    }
  }
}
