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
  Layout.minimumWidth: 500
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
      Layout.minimumWidth: 300
      anchors.bottom: parent.bottom
      anchors.top: parent.top
      anchors.left: parent.left
      height: parent.height
      ColumnLayout {
        id: localColumn
        Layout.minimumHeight: 100
        Layout.fillWidth: true
        Rectangle {
          color: evenColor
          Layout.alignment: Qt.AlignLeft
          Layout.preferredHeight: 20
          Layout.fillWidth: true
          Label {
            text: "Local resources"
            anchors.fill: parent
            font.pointSize: 12
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
              id: item
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
                y: item.z - 30
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
        Rectangle {
          color: evenColor
          Layout.alignment: Qt.AlignLeft
          Layout.preferredHeight: 20
          Layout.fillWidth: true
          Label {
            text: "Fuel resources"
            anchors.fill: parent
            font.pointSize: 12
          }
        }
        TreeView {
          id: treeView2
          model: OwnerList
          // For some reason, SingleSelection is not working
          selectionMode: SelectionMode.MultiSelection
          verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
          headerVisible: false
          backgroundVisible: false
          Layout.minimumWidth: 300
          Layout.alignment: Qt.AlignCenter
          Layout.fillWidth: true
          Layout.fillHeight: true

          headerDelegate: Rectangle {
            visible: false
          }

          TableViewColumn
          {
            role: "name"
          }

          selection: ItemSelectionModel {
            model: OwnerList
          }

          style: TreeViewStyle {
            indentation: 0
            rowDelegate: Rectangle {
              id: row2
              color: Material.background
              height: treeItemHeight
            }
            itemDelegate: Rectangle {
              id: item
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
                  ResourceSpawner.OnOwnerClicked(model.path)
                  treeView2.selection.select(styleData.index, ItemSelectionModel.ClearAndSelect)
                  treeView.selection.clearSelection()
                  currentPath = model.path
                  gridView.currentIndex = -1
                  mouse.accepted = false
                }
              }

              ToolTip {
                visible: ma.containsMouse
                delay: 500
                y: item.z - 30
                text: model === null ?
                "?" : model.path
                enter: null
                exit: null
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
        Layout.fillWidth: true
        Layout.minimumWidth: 300
        height: 40
        color: evenColor
        Label {
          text: currentPath
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
                        (model.thumbnail == "" ?
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
              Rectangle {
                color: "transparent"
                anchors.fill: parent
                Label {
                  width: downloadDialog.width - 50
                  height: downloadDialog.height
                  text: "Please download the model first by clicking the cloud icon."
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
                ResourceSpawner.OnDownloadFuelResource(model.sdf, model.index)
                model.isDownloaded = true
              }
            }
          }
        }
      }
    }
  }
}
