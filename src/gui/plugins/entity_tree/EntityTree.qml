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
import QtQml.Models 2.2
import QtQuick

import QtQuick.Controls
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3

import QtQuick.Dialogs
import GzSim 1.0 as GzSim

Rectangle {
  id: entityTree
  color: lightGrey
  Layout.minimumWidth: 400
  Layout.minimumHeight: 375
  anchors.fill: parent

  /**
   * Time delay for tooltip to show, in ms
   */
  property int tooltipDelay: 500

  /**
   * Dark grey according to theme
   */
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  /**
   * Light grey according to theme
   */
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Highlight color
   */
  property color highlightColor: Qt.rgba(
    Material.accent.r,
    Material.accent.g,
    Material.accent.b, 0.3)

  /**
   * Height of each item in pixels
   */
  property int itemHeight: 30

  /**
   * Color for even-numbered rows, according to current theme
   */
  property color even: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Color for odd-numbered rows, according to current theme
   */
  property color odd: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  /*
   * Deselect all entities.
   */
  function deselectAllEntities() {
    tree.selectionModel.clear()
  }

  /*
   * Iterate through item's children until the one corresponding to _entity is
   * found and select that.
   */
  function selectFromCpp(_entity, itemId) {
    if (_EntityTreeModel.data(itemId, 101) == _entity) {
      tree.selectionModel.select(itemId, ItemSelectionModel.Select)
      return
    }
    for (var i = 0; i < _EntityTreeModel.rowCount(itemId); i++) {
      selectFromCpp(_entity, _EntityTreeModel.index(i, 0, itemId))
    }
  }

  /*
   * Callback when an entity selection comes from the C++ code.
   * For example, if it comes from the 3D window.
   */
  function onEntitySelectedFromCpp(_entity) {
    for(var i = 0; i < _EntityTreeModel.rowCount(); i++) {
      var itemId = _EntityTreeModel.index(i, 0)
      selectFromCpp(_entity, itemId)
    }
  }

  // The component for a menu section header
  Component {
    id: menuSectionHeading
    Rectangle {
      height: childrenRect.height

      Text {
          text: sectionText
          font.pointSize: 10
          padding: 5
      }
    }
  }

  Rectangle {
    id: header
    visible: true
    height: addEntity.height
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    width: parent.width
    color: darkGrey

    RowLayout {
      anchors.fill: parent
      spacing: 0

      Label {
        text: "Entity Tree"
        font.capitalization: Font.Capitalize
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 5
      }

      ToolButton {
        Layout.alignment: Qt.AlignVCenter | Qt.AlignRight
        id: addEntity
        ToolTip.text: "Add an entity to the world"
        ToolTip.visible: hovered
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: "qrc:/Gazebo/images/plus.png"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        onClicked: addEntityMenu.open()

        FileDialog {
          id: loadFileDialog
          title: "Load mesh"
          nameFilters: [ "Supported mesh files (*.dae *.fbx *.glb *.gltf *.obj *.stl *.stla *.stlb)" ]
          fileMode: FileDialog.OpenFile
          currentFolder: StandardPaths.writableLocation(StandardPaths.HomeLocation)
          onAccepted: {
            _EntityTree.OnLoadMesh(selectedFile)
          }
        }

        Menu {
          id: addEntityMenu

          Item {
            Layout.fillWidth: true
            height: childrenRect.height
            Loader {
              property string sectionText: "Model"
              sourceComponent: menuSectionHeading
            }
          }

          MenuItem
          {
            id: box
            text: "Box"
            onClicked: {
              _EntityTree.OnInsertEntity("box")
            }
          }

          MenuItem
          {
            id: capsule
            text: "Capsule"
            onClicked: {
              _EntityTree.OnInsertEntity("capsule")
            }
          }

          MenuItem
          {
            id: cone
            text: "Cone"
            onClicked: {
              _EntityTree.OnInsertEntity("cone")
            }
          }

          MenuItem
          {
            id: cylinder
            text: "Cylinder"
            onClicked: {
              _EntityTree.OnInsertEntity("cylinder")
            }
          }

          MenuItem
          {
            id: ellipsoid
            text: "Ellipsoid"
            onClicked: {
              _EntityTree.OnInsertEntity("ellipsoid")
            }
          }

          MenuItem
          {
            id: sphere
            text: "Sphere"
            onClicked: {
              _EntityTree.OnInsertEntity("sphere")
            }
          }

          MenuItem
          {
            id: mesh
            text: "Mesh"
            onClicked: {
              loadFileDialog.open()
            }
          }

          MenuSeparator {
            padding: 0
            topPadding: 12
            bottomPadding: 12
            contentItem: Rectangle {
              implicitWidth: 200
              implicitHeight: 1
              color: "#1E000000"
            }
          }

          Item {
            Layout.fillWidth: true
            height: childrenRect.height
            Loader {
              property string sectionText: "Light"
              sourceComponent: menuSectionHeading
            }
          }

          MenuItem
          {
            id: directionalLight
            text: "Directional"
            onClicked: {
              _EntityTree.OnInsertEntity("directional")
            }
          }

          MenuItem
          {
            id: pointLight
            text: "Point"
            onClicked: {
              _EntityTree.OnInsertEntity("point")
            }
          }

          MenuItem
          {
            id: spotLight
            text: "Spot"
            onClicked: {
              _EntityTree.OnInsertEntity("spot")
            }
          }
        }
      }
    }
  }

  TreeView {
    id: tree
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    model: _EntityTreeModel

    selectionModel: ItemSelectionModel {
      model: _EntityTreeModel
    }
    palette {
      base: lightGrey
      alternateBase: darkGrey
      highlight: highlightColor
      windowText: "black"
    }

    delegate: TreeViewDelegate {
      id: treeDelegate
      implicitWidth: entityTree.width
      leftMargin: 0
      rightMargin: 0
      indentation: 20
      spacing: 0

      indicator: Rectangle {
        x: leftMargin + (depth * indentation)
        height: itemHeight
        width: itemHeight * 0.75
        anchors.margins: 0
        color:  "transparent"
        Image {
          id: indicatorIcon
          anchors.margins: 0
          sourceSize.height: itemHeight * 0.4
          sourceSize.width: itemHeight * 0.4
          fillMode: Image.Pad
          anchors.verticalCenter: parent.verticalCenter
          anchors.right: parent.right
          source: tree.isExpanded(row) ?
              "qrc:/Gazebo/images/chevron-down.svg" : "qrc:/Gazebo/images/chevron-right.svg"
        }
        MouseArea {
          anchors.fill: parent
          hoverEnabled: true
          propagateComposedEvents: true
          onClicked: mouse => {
            // Stop the event propagation. The TreeView's own collapsible
            // behaviour gets messy otherwise.
            mouse.accepted = true
            if (tree.isExpanded(row))
              tree.collapse(row)
            else
              tree.expand(row)
          }
        }
      }

      contentItem: Rectangle {
        id: itemDel
        color: "transparent"
        implicitHeight: itemHeight
        anchors.margins: 0

        GzSim.TypeIcon {
          id: icon
          height: itemHeight - 5
          width: itemHeight - 5
          entityType: model === null || model.type === undefined ? "" : model.type
          anchors.margins: 0
        }

        Text {
          anchors.verticalCenter: parent.verticalCenter
          anchors.left: icon.right
          leftPadding: 2
          topPadding: 0
          bottomPadding: 0
          text: model === null || model.entityName === undefined ? "" : model.entityName
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        ToolTip {
          visible: ma.containsMouse
          delay: tooltipDelay
          text: model === null || model.entity === undefined ?
              "Entity Id: ?" : "Entity Id: " + model.entity
          y: itemDel.z - 30
          enter: null
          exit: null
        }

        MouseArea {
          id: ma
          anchors.fill: parent
          hoverEnabled: true
          propagateComposedEvents: true
          acceptedButtons: Qt.RightButton | Qt.LeftButton
          onClicked: mouse => {
            mouse.accepted = false
            if (mouse.button == Qt.RightButton) {
              var mi = treeDelegate.treeView.modelIndex(Qt.point(column, row))
              var type = _EntityTreeModel.EntityType(mi)
              var scopedName = _EntityTreeModel.ScopedName(mi)
              var posInTree = mapToItem(entityTree, ma.mouseX, ma.mouseY)
              entityContextMenu.open(scopedName, type, posInTree.x, posInTree.y)
              // Prevent plugin's context menu from opening
              mouse.accepted = true
            }
            else if (mouse.button == Qt.LeftButton) {
              var mi = treeDelegate.treeView.modelIndex(Qt.point(column, row))
              var mode = mouse.modifiers & Qt.ControlModifier ?
                  ItemSelectionModel.Select : ItemSelectionModel.ClearAndSelect
              var entity = _EntityTreeModel.EntityId(mi)
              _EntityTree.OnEntitySelectedFromQml(entity)
              tree.selectionModel.select(mi, mode)
            }
          }
        }
      }
    }

    GzSim.EntityContextMenu {
      id: entityContextMenu
    }
  }
}
