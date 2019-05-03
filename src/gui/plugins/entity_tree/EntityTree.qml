import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: entityTree
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375
  anchors.fill: parent

  property int tooltipDelay: 500
  property int tooltipTimeout: 1000
  property int itemHeight: 30

  property color even: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  property color odd: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  TreeView {
    anchors.fill: entityTree
    model: EntityTreeModel

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
        height: itemHeight
        color: (styleData.row % 2 == 0) ? even : odd
      }

      itemDelegate: Rectangle {
        id: itemDel
        color: (styleData.row % 2 == 0) ? even : odd
        height: itemHeight

        Image {
          id: icon
          sourceSize.height: itemHeight
          sourceSize.width: itemHeight
          fillMode: Image.PreserveAspectFit
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignLeft
          source: model === null || model.icon === undefined ? "" : model.icon
        }

        Text {
          anchors.verticalCenter: parent.verticalCenter
          anchors.left: icon.right
          leftPadding: 2
          text: model === null || model.entityName === undefined ? "" : model.entityName
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12

          ToolTip {
            visible: ma.containsMouse
            delay: tooltipDelay
            timeout: tooltipTimeout
            text: model === null || model.entity === undefined ? "" : model.entity
            y: itemDel.z - 30
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
      role: "entityName"
      width: 300
    }
  }
}
