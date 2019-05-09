import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: componentInspector
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
    model: ComponentInspectorModel

    // Hacky: the sibling of listView is the background(Rectangle) of TreeView
    Component.onCompleted: {
      tree.__listView.parent.children[1].color = Material.background
    }
    Material.onThemeChanged: {
      tree.__listView.parent.children[1].color = Material.background
    }

    style: TreeViewStyle {
      headerDelegate: Rectangle {
        visible: false
      }

      branchDelegate: Rectangle {
        height: itemHeight
        width: itemHeight*0.5
        color: "transparent"
        Text {
          font.pointSize: 18
          font.family: "Roboto"
          anchors.verticalCenter: parent.verticalCenter
          anchors.horizontalCenter: parent.horizontalCenter
          text: styleData.isExpanded ? "\uFF0D" : "\uFF0B"
          color: Material.theme == Material.Light ? "black" : "white"
        }
      }

      rowDelegate: Rectangle {
        visible: styleData.row !== undefined
        height: itemHeight
        color: (styleData.row % 2 == 0) ? even : odd
      }

      itemDelegate: Rectangle {
        id: itemDel
        color: (styleData.row % 2 == 0) ? even : odd
        height: itemHeight

        Text {
          anchors.verticalCenter: parent.verticalCenter
          leftPadding: 2
          text: (model === null) ? "" :
                (model.typeName !== undefined) ? model.typeName :
                (model.x !== undefined) ?
                  (model.x + " " +
                  model.y + " " +
                  model.z + " " +
                  model.roll + " " +
                  model.pitch + " " +
                  model.yaw) : ""
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12

          ToolTip {
            visible: ma.containsMouse
            delay: tooltipDelay
            text: model === null || model.typeId === undefined ?
                "Type Id: ?" : "Type Id: " + model.typeId
            y: itemDel.z - 30
            enter: null
            exit: null
          }
          MouseArea {
            id: ma
            anchors.fill: parent
            hoverEnabled: true
          }
        }
      }
    }

    TableViewColumn {
      role: "typeName"
      width: 300
    }
  }
}
