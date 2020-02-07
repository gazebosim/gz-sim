import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: stringComponent
  height: typeHeader.height
  width: componentInspector.width
  color: "transparent"

  function textFromModel(_model) {
    if (_model && _model.data !== undefined && typeof _model.data == 'string')
      return _model.data

    return 'N/A'
  }

  Row {
    spacing: stringComponent.width - typeHeader.width - content.width - 20
    TypeHeader {
      id: typeHeader
    }

    TextInput {
      id: content
      text: textFromModel(model)
      color: Material.theme == Material.Light ? "black" : "white"
      font.pointSize: 12
      selectByMouse: true // will only work when we enable it
      enabled: false
    }
  }
}
