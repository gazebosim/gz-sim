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
  Layout.minimumWidth: 500
  Layout.minimumHeight: 400
  anchors.fill: parent
  orientation: Qt.Horizontal

  property int treeItemHeight: 30;

  property int gridItemHeight: 150;

  property int gridItemWidth: 200;

  property string currentPath: "";

  property color oddColor: (Material.theme == Material.Light) ?
  Material.color(Material.Grey, Material.Shade100):
  Material.color(Material.Grey, Material.Shade800);

  property color evenColor: (Material.theme == Material.Light) ?
  Material.color(Material.Grey, Material.Shade200):
  Material.color(Material.Grey, Material.Shade900);

  function windowWidth() {
    return resourceView.Window.window ? (resourceView.Window.window.width) : 0
  }

  function windowHeight() {
    return resourceView.Window.window ? (resourceView.Window.window.height) : 0
  }

  SplitView {
    orientation: Qt.Vertical
    Layout.fillWidth: true
    Layout.minimumHeight: 400
    Layout.minimumWidth: 300
    ColumnLayout {
      spacing: 2
      Layout.preferredHeight: resourceView.height / 2
      Rectangle {
        Layout.alignment: Qt.AlignCenter
        Layout.preferredWidth: resourceView.width
        Layout.preferredHeight: 35
        Text {
          text: "Local models"
          font.pointSize: 15
          leftPadding: 5
          topPadding: 5
        }
      }
      TreeView {
        Layout.alignment: Qt.AlignCenter
        Layout.preferredWidth: resourceView.width
        id: treeView
        model: PathList
        // For some reason, SingleSelection is not working
        selectionMode: SelectionMode.MultiSelection
        verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
        headerVisible: false
        backgroundVisible: false
        Layout.minimumWidth: 300

        headerDelegate: Rectangle {
          visible: false
        }

  /**
   * Height of each item on the tree
   */
  property int treeItemHeight: 30;

  /**
   * Height of each item on the grid
   */
  property int gridItemHeight: 150;

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

    headerDelegate: Rectangle {
      visible: false
    }

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
                treeView.selection.select(styleData.index, ItemSelectionModel.ClearAndSelect)
                treeView2.selection.clearSelection()
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

    ColumnLayout {
      spacing: 2
      anchors.bottom: resourceView.bottom
      Layout.preferredHeight: resourceView.height / 2
      Rectangle {
        Layout.alignment: Qt.AlignCenter
        Layout.preferredWidth: resourceView.width
        Layout.preferredHeight: 35
        Text {
          text: "Fuel models"
          font.pointSize: 15
          leftPadding: 5
          topPadding: 5
        }
      }
      TreeView {
        id: treeView2
        model: OwnerList
        // For some reason, SingleSelection is not working
        selectionMode: SelectionMode.MultiSelection
        verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
        Layout.preferredWidth: resourceView.width
        headerVisible: false
        backgroundVisible: false
        Layout.minimumWidth: 300

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
        anchors.margins: 5
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
              source: (model.isFuel && !model.isDownloaded) ? "DownloadToUse.png" : (model.thumbnail == "" ? "NoThumbnail.png" : "file:" + model.thumbnail)
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
                print("current index")
                print(gridView.currentIndex)
                print("index")
                print(index)
              }
              print("resource clicked");
            }
          }
          Dialog {
            id: downloadDialog
            parent: resourceView.Window.window ? resourceView.Window.window.contentItem : resourceView
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
              Text {
                width: downloadDialog.width - 50
                height: downloadDialog.height
                text: "Please download the model first by clicking the cloud icon in order to drag it into the scene."
                color: Material.theme == Material.Light ? "black" : "white"
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
          Image {
            id: download
            source: (model.isFuel && !model.isDownloaded) ? "CloudDownload.png" : ""
            anchors.bottom: parent.bottom
            anchors.right: parent.right
            fillMode: Image.PreserveAspectFit
            height: thumbnailImage.height / 4
            width: thumbnailImage.width / 4
            verticalAlignment: Image.AlignRight
            horizontalAlignment: Image.AlignBottom
          }
          MouseArea {
            anchors.fill: download
            hoverEnabled: true
            propagateComposedEvents: true
            onClicked: {
              print("download clicked");
              ResourceSpawner.OnDownloadFuelResource(model.sdf, model.index)
              model.isDownloaded = true
            }
            Image {
              Layout.fillHeight: true
              Layout.fillWidth: true
              Layout.margins: 1
              source: model.thumbnail == "" ? "NoThumbnail.png" : model.thumbnail
              fillMode: Image.PreserveAspectFit
            }
          }
          MouseArea {
            id: ma
            anchors.fill: parent
            hoverEnabled: true
            propagateComposedEvents: true
            onClicked: {
              ResourceSpawner.OnResourceSpawn(model.sdf);
              gridView.currentIndex = index
            }
          }
          ToolTip {
            visible: ma.containsMouse
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
