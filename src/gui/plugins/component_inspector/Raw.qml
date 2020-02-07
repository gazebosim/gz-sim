import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: stringComponent
  height: header.height + content.height
  width: componentInspector.width
  color: "transparent"

  function textFromModel(_model) {
    if (_model && _model.data !== undefined && typeof _model.data == 'string')
      return _model.data

    return 'N/A'
  }

  Column {
    anchors.fill: parent

    Rectangle {
      id: header
      width: parent.width
      height: typeHeader.height
      color: "transparent"

      TypeHeader {
        id: typeHeader
      }
      MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: {
          content.show = !content.show
        }
        onEntered: {
          header.color = darkGrey
        }
        onExited: {
          header.color = "transparent"
        }
      }
    }

    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? textArea.height : 0
      clip: true
      color: darkGrey

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      TextArea {
        id: textArea
        width: parent.width
        text: textFromModel(model)
        color: Material.theme == Material.Light ? "black" : "white"
        font.pointSize: 12
        selectByMouse: true // will only work when we enable it
        enabled: false
      }
    }
  }
}
