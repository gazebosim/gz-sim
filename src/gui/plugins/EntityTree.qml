import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

Rectangle {
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375
  Material.theme: Material.theme

  // TODO(louise) It's not responding to theme changes
  property color even: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade600)

  property color odd: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade300) :
    Material.color(Material.Grey, Material.Shade800)

  TreeView {
    width: 300
    height: 500
    model: EntityTreeModel

    style: TreeViewStyle {
      headerDelegate: Rectangle {
        visible: false
      }

      branchDelegate: Rectangle {
        height: 20
        width: 20
        color: "transparent"
        Text {
          font.pointSize: 18
          anchors.verticalCenter: parent.verticalCenter
          anchors.horizontalCenter: parent.horizontalCenter
          text: styleData.isExpanded ? "\uFF0D" : "\uFF0B"
          color: Material.theme == Material.Light ? "black" : "white"
        }
      }

      rowDelegate: Rectangle {
        height: 20
        color: (styleData.row % 2 == 0) ? even : odd
      }

      itemDelegate: Rectangle {
        color: (styleData.row % 2 == 0) ? even : odd
        height: 20

        Text {
          anchors.verticalCenter: parent.verticalCenter
          text: styleData.value === undefined ? "" : styleData.value
          color: Material.theme == Material.Light ? "black" : "white"
        }
      }
    }

    TableViewColumn {
      role: "entityName"
      width: 300
    }
  }
}
