import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: itemDel
  color: (styleData.row % 2 == 0) ? even : odd
  height: itemHeight

  function tooltipText(_model) {
    if (model === null)
      return "Unknown component"

    var info = '';
    if (model.typeName !== undefined)
      info += "Type name [" + model.typeName + "]\n"

    if (model.typeId !== undefined)
      info += "Type Id [" + model.typeId + "]"

    return info
  }

  Text {
    anchors.verticalCenter: parent.verticalCenter
    leftPadding: 2
    text: model && model.shortName ? model.shortName : ''
    color: Material.theme == Material.Light ? "black" : "white"
    font.pointSize: 12

    ToolTip {
      visible: ma.containsMouse
      delay: tooltipDelay
      text: tooltipText(model)
      y: itemDel.z - 30
      enter: null
      exit: null
    }
    MouseArea {
      id: ma
      anchors.fill: parent
      hoverEnabled: true
      acceptedButtons: Qt.RightButton
    }
  }
}
