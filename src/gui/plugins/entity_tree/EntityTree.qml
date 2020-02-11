import QtQml.Models 2.2
import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import IgnGazebo 1.0 as IgnGazebo

Rectangle {
  id: entityTree
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375
  anchors.fill: parent

  /**
   * Time delay for tooltip to show, in ms
   */
  property int tooltipDelay: 500

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

  TreeView {
    id: tree
    anchors.fill: parent
    model: EntityTreeModel
    selectionMode: SelectionMode.MultiSelection

    // Hacky: the sibling of listView is the background(Rectangle) of TreeView
    Component.onCompleted: {
      tree.__listView.parent.children[1].color = Material.background
    }
    Material.onThemeChanged: {
      tree.__listView.parent.children[1].color = Material.background
    }

    selection: ItemSelectionModel {
      model: EntityTreeModel
    }

    style: TreeViewStyle {
      indentation: itemHeight * 0.75

      headerDelegate: Rectangle {
        visible: false
      }

      branchDelegate: Rectangle {
        height: itemHeight
        width: itemHeight * 0.75
        color: "transparent"
        Image {
          id: icon
          sourceSize.height: itemHeight * 0.4
          sourceSize.width: itemHeight * 0.4
          fillMode: Image.Pad
          anchors.verticalCenter: parent.verticalCenter
          anchors.right: parent.right
          source: styleData.isExpanded ?
              "qrc:/Gazebo/images/minus.png" : "qrc:/Gazebo/images/plus.png"
        }
      }

      rowDelegate: Rectangle {
        visible: styleData.row !== undefined
        height: itemHeight
        color: styleData.selected ? Material.accent : (styleData.row % 2 == 0) ? even : odd
      }

      itemDelegate: Rectangle {
        id: itemDel
        color: styleData.selected ? Material.accent : (styleData.row % 2 == 0) ? even : odd
        height: itemHeight

        Image {
          id: icon
          sourceSize.height: itemHeight
          sourceSize.width: itemHeight
          fillMode: Image.PreserveAspectFit
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignLeft
          source: model === null || model.icon === undefined ? "" : model.icon

          ToolTip {
            visible: iconMa.containsMouse
            delay: tooltipDelay
            text: model === null || model.type === undefined ? "" : model.type
            y: icon.z - 30
            enter: null
            exit: null
          }
          MouseArea {
            id: iconMa
            anchors.fill: parent
            hoverEnabled: true
          }
        }

        Text {
          anchors.verticalCenter: parent.verticalCenter
          anchors.left: icon.right
          leftPadding: 2
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
          onClicked: {
            if (mouse.button == Qt.RightButton) {
              var type = EntityTreeModel.EntityType(styleData.index)
              var scopedName = EntityTreeModel.ScopedName(styleData.index)
              entityContextMenu.open(scopedName, type, ma.mouseX, ma.mouseY)
            }
            else if (mouse.button == Qt.LeftButton) {
              var entity = EntityTreeModel.EntityId(styleData.index)
              EntityTree.OnEntitySelectedFromQml(entity)
              tree.selection.select(styleData.index, ItemSelectionModel.Select)
            }
            mouse.accepted = false
          }

          IgnGazebo.EntityContextMenu {
            id: entityContextMenu
            anchors.fill: parent
          }
        }
      }
    }

    TableViewColumn {
      role: "entityName"
      width: 300
    }
  }

  /*
   * Deselect all entities.
   */
  function deselectAllEntities() {
    tree.selection.clear()
  }

  /*
   * Iterate through item's children until the one corresponding to _entity is
   * found and select that.
   */
  function selectRecursively(_entity, itemId) {
    if (EntityTreeModel.data(itemId, 101) == _entity) {
      tree.selection.select(itemId, ItemSelectionModel.Select)
      return
    }
    for (var i = 0; i < EntityTreeModel.rowCount(itemId); i++) {
      selectRecursively(_entity, EntityTreeModel.index(i, 0, itemId))
    }
  }

  /*
   * Callback when an entity selection comes from the C++ code.
   * For example, if it comes from the 3D window.
   */
  function onEntitySelectedFromCpp(_entity) {
    for(var i = 0; i < EntityTreeModel.rowCount(); i++) {
      var itemId = EntityTreeModel.index(i, 0)
      selectRecursively(_entity, itemId)
    }
  }
}
