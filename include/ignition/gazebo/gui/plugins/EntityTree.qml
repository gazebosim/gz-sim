import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Layouts 1.3

Rectangle {
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375

  TreeView {
    width: 300
    height: 500
    model: EntityTreeModel
    itemDelegate: Rectangle {
      color: "white"
      height: 20

      Text {
        anchors.verticalCenter: parent.verticalCenter
        text: styleData.value === undefined ? "" : styleData.value
      }
    }

    TableViewColumn {
      role: "entityName"
      title: "Entities"
      width: 300
    }
  }
}
