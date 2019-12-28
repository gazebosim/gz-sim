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

  function textFromModel(_model) {
    if (_model && _model.name !== undefined)
      return _model.name

    return ''
  }

  Text {
    anchors.verticalCenter: parent.verticalCenter
    leftPadding: 2
    text: textFromModel(model)
    color: Material.theme == Material.Light ? "black" : "white"
    font.pointSize: 12
  }
}
