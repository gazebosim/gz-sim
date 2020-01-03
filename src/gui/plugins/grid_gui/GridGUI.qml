import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3

Rectangle {
  id: gridconfig
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375

  property int tooltipDelay: 500
  property int tooltipTimeout: 1000

  Column {
    anchors.fill: parent
    anchors.margins: 10

    CheckBox {
      text: qsTr("Show/Hide")
      checked: true
    }
    
    Label {
      text: "vertical cell count"
    }

    SpinBox {
      from: 1
      to: 50
    }

    Label {
      text: "horizontal cell count"
    }

    SpinBox {
      from: 1
      to: 50
    }
  }
}
