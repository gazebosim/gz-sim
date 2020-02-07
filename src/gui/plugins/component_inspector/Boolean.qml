import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: booleanComponent
  height: header.height
  width: componentInspector.width
  color: "transparent"

  Row {
    spacing: booleanComponent.width - header.width - content.width - 20
    TypeHeader {
      id: header
    }

    Switch {
      id: content
      y: -content.width * 0.2
      checked: model.data
      enabled: false
    }
  }
}
