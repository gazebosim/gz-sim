import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

import QtQml.Models 2.2

SplitView {
  id: resourceView
  // anchors.fill: parent
  orientation: Qt.Horizontal

  TreeView {
    id: treeView
    model: PathList
    Layout.fillWidth: true
    Layout.minimumHeight: 400
    Layout.minimumWidth: 300
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
        width: parent.width

        anchors.top: parent.top
        anchors.right: parent.right

        Image {
          id: dirIcon
          source: "copy_object.png"
          height: treeItemHeight * 0.6
          width: treeItemHeight * 0.6
          anchors.verticalCenter: parent.verticalCenter
          anchors.left: parent.left
        }

        Text {
          width: parent.width - 10
          text: (model === null) ? "" : model.path
          color: Material.theme == Material.Light ? "black" : "white"
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

  ColumnLayout {
    Layout.minimumWidth: 300
    Layout.fillWidth: true
    spacing: 0
    Rectangle {
      Layout.fillWidth: true
      Layout.minimumWidth: 300
      height: 40
      color: evenColor
      Text {
        id: currentPathText
        text: currentPath
        font.pointSize: 12
        elide: Text.ElideMiddle
        anchors.margins: 5
        anchors.verticalCenter: parent.verticalCenter
        anchors.left: parent.left
        anchors.right: parent.right
        color: Material.theme == Material.Light ? "black" : "white"
      }
    }
    Rectangle {
      Layout.fillHeight: true
      Layout.fillWidth: true
      color: Material.background
      GridView {
        id: gridView
        model: LocalModelList
        cellWidth: gridItemWidth
        cellHeight: gridItemHeight
        currentIndex: -1
        clip: true
        anchors.fill: parent
        anchors.margins: 20
        delegate: Pane {
          id: itemDelegate
          width: gridView.cellWidth - 8
          height: gridView.cellHeight - 8
          Material.elevation: 6
          background: Rectangle {
            color: gridView.currentIndex === index ? Material.accent : Material.background
            layer.enabled: true
            layer.effect: ElevationEffect {
              elevation: 6
            }
          }

          ColumnLayout {
            anchors.fill: parent
            Text {
              text: model.name
              color: Material.theme == Material.Light ? "black" : "white"
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
              source: model.thumbnail == "" ? "NoThumbnail.png" : "file:" + model.thumbnail
              fillMode: Image.PreserveAspectFit
            }
          }
          MouseArea {
            id: ma2
            anchors.fill: parent
            hoverEnabled: true
            propagateComposedEvents: true
            onClicked: {
              ResourceSpawner.OnResourceSpawn(model.sdf);
              gridView.currentIndex = index;
              print("resource clicked");
            }
          }
          ToolTip {
            visible: ma2.containsMouse
            delay: 500
            text: model === null ? "N/A" : model.name
            enter: null
            exit: null
          }
        }
      }
    }
  }
}
