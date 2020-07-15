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
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

import QtQml.Models 2.2

Rectangle {
  id: rectangle
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375
  anchors.fill: parent
  SplitView {
    anchors.fill: parent
    orientation: Qt.Horizontal
    Rectangle {
      width: 197
      Layout.maximumWidth: parent.width - 50
      Layout.minimumWidth: 50
      color: "transparent"
      TreeView {
        id: treeView
        anchors.fill: parent
        model: PathList
        Layout.minimumHeight: 400
        Layout.minimumWidth: 300
        selectionMode: SelectionMode.SingleSelection
        verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
        headerVisible: false
        backgroundVisible: false

        property int itemHeight: 30;

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

        property color oddColor: (Material.theme == Material.Light) ?
        Material.color(Material.Grey, Material.Shade100):
        Material.color(Material.Grey, Material.Shade800);

        property color evenColor: (Material.theme == Material.Light) ?
        Material.color(Material.Grey, Material.Shade200):
        Material.color(Material.Grey, Material.Shade900);

        property color highlightColor: Material.accentColor;

        style: TreeViewStyle {
          rowDelegate: Rectangle {
            id: row
            color: (styleData.selected)? Material.accent :
            (styleData.row % 2 == 0) ? treeView.evenColor : treeView.oddColor
            height: treeView.itemHeight

          }
          itemDelegate: Rectangle {
            id: item
            color: styleData.selected ? Material.accent : (styleData.row % 2 == 0) ? treeView.evenColor : treeView.oddColor
            height: treeView.itemHeight

            anchors.top: parent.top
            anchors.right: parent.right

            Image {
              id: dirIcon
              source: "copy_object.png"
              height: treeView.itemHeight * 0.6
              width: treeView.itemHeight * 0.6
              x: -10
              y: treeView.itemHeight * 0.2
            }

            MouseArea {
              id: ma
              anchors.fill: parent
              propagateComposedEvents: true
              hoverEnabled: true
              onClicked: {
                ResourceSpawner.OnPathClicked(model.path);
                mouse.accepted = false
                var mode = mouse.modifiers & ItemSelectionModel.Select
                treeView.selection.select(styleData.index, mode)
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

            Text {
              id : field
              width: parent.width
              text: (model === null) ? "" : model.path
              color: Material.theme == Material.Light ? "black" : "white"
              elide: Text.ElideRight
              font.pointSize: 12
              anchors.leftMargin: 1
              anchors.left: dirIcon.right
              leftPadding: 2
              y: dirIcon.y
            }
          }
        }
      }
    }
    Rectangle {
      id: rightItem
      Layout.minimumWidth: 50
      Layout.fillWidth: true
      color: "transparent"
      x: 10
      GridView {
        id: gridView
        anchors.fill: parent
        model: LocalModelList
        cellWidth: 200
        cellHeight: 200
        delegate: Rectangle {
          id: rectDel
          color: "transparent"
          Pane {
            width: gridView.cellWidth - 8
            height: gridView.cellHeight - 8
            Material.elevation: 6
            Image {
              anchors.fill: parent
              anchors.margins: 1
              source: model.thumbnail == "" ? "NoThumbnail.png" : model.thumbnail
              fillMode: Image.Stretch
            }
            MouseArea {
              id: ma
              anchors.fill: parent
              hoverEnabled: true
              propagateComposedEvents: true
              onClicked: {
                ResourceSpawner.OnResourceSpawn(model.sdf);
              }
            }
            ToolTip {
              visible: ma.containsMouse
              delay: 500
              y: rectDel.z - 30
              text: model === null ?
              "?" : model.name
              enter: null
              exit: null
            }
            Text {
              text: model.name
              color: "#444444"
              width: parent.width
              height: parent.height
              font.pointSize: 12
              elide: Text.ElideRight
              anchors.bottom: parent.bottom
              anchors.horizontalCenter: parent.horizontalCenter
            }
          }
        }
      }
    }
  }
}
