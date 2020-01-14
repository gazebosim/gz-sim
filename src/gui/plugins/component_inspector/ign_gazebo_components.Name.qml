import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: nameComponent
  height: header.height
  width: componentInspector.width
  color: "transparent"

  function textFromModel(_model) {
    if (_model && _model.data !== undefined && typeof _model.data == 'string')
      return _model.data

    return 'N/A'
  }

  Row {
    spacing: nameComponent.width - header.width - content.width - 20
    TypeHeader {
      id: header
    }

    Text {
      id: content
      text: textFromModel(model)
      color: Material.theme == Material.Light ? "black" : "white"
      font.pointSize: 12
    }
  }
}
