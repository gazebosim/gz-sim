import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"
import "qrc:/qml"

Rectangle {
  id: numberComponent
  height: typeHeader.height
  width: componentInspector.width
  color: "transparent"

  // Maximum spinbox value
  property double spinMax: 1000000

  Row {
    spacing: numberComponent.width - typeHeader.width - content.width - 20
    TypeHeader {
      id: typeHeader
    }

    IgnSpinBox {
      id: content
      value: model.data
      minimumValue: -spinMax
      maximumValue: spinMax
      decimals: xSpin.width < 100 ? 2 : 6
      Layout.fillWidth: true
    }
  }
}
