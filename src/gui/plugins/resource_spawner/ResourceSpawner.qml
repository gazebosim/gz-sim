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
      width: 200
      Layout.maximumWidth: 400
      color: "transparent"
      TreeView {
        id: treeView
        anchors.fill: parent
        model: PathList
        Layout.minimumHeight: 400
        Layout.minimumWidth: 300
        property int itemHeight: 30;

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

        // =========== Colors ===========
        property color oddColor: (Material.theme == Material.Light) ?
        Material.color(Material.Grey, Material.Shade100):
        Material.color(Material.Grey, Material.Shade800);

        property color evenColor: (Material.theme == Material.Light) ?
        Material.color(Material.Grey, Material.Shade200):
        Material.color(Material.Grey, Material.Shade900);

        property color highlightColor: Material.accentColor;

        rowDelegate: Rectangle {
          id: row
          color: (styleData.selected)? treeView.highlightColor :
          (styleData.row % 2 == 0) ? treeView.evenColor : treeView.oddColor
          height: treeView.itemHeight
        }
        itemDelegate: Item {
          id: item
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

          Text {
            id : field
            width: parent.width
            text: (model === null) ? "" : model.path
            color: (Material.theme == Material.Light || styleData.selected) ?
            Material.color(Material.Grey, Material.Shade800):
            Material.color(Material.Grey, Material.Shade400);
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
    Rectangle {
      id: centerItem
      Layout.minimumWidth: 50
      Layout.fillWidth: true
      color: "lightgray"
      Text {
        text: "View 2"
        anchors.centerIn: parent
      }
    }
    /*
     ListView {
       id: listView
       anchors.fill: parent
       model: PathListi
       delegate: Rectangle {
         Text {
           text: model.path
         }
       }
     }
     GridView {
       id: gridView
       anchors.fill: parent
       model: LocalModelList
       delegate: Rectangle {
         width: gridView.cellWidth
         height: gridView.cellHeight
         color: "transparent"
         Pane {
           width: parent.width
           height: parent.height
           Material.elevation: 6
           Image {
             anchors.fill: parent
             anchors.margins: 1
             source: model.thumbnail == "" ? "NoThumbnail.png" : model.thumbnail
             fillMode: Image.PreserveAspectFit
           }
           MouseArea {
             anchors.fill: parent
             onClicked: {
               ResourceSpawner.OnResourceSpawn(model.sdf);
             }
           }
           Text {
             text: model.name
             color: Material.theme == Material.Light ? "#444444" : "#cccccc"
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
     */
  }
}
