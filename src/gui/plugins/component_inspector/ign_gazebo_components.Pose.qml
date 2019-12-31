import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: "transparent"

  function textFromModel(_model) {
    if (_model &&_model.x)
      return (_model.x + " " +
              _model.y + " " +
              _model.z + " " +
              _model.roll + " " +
              _model.pitch + " " +
              _model.yaw)

    return 'N/A'
  }

  Column {
    anchors.fill: parent

    Rectangle {
      id: header
      width: parent.width
      height: headerItem.height
      color: "transparent"

      TypeHeader {
        id: headerItem
      }
      MouseArea {
        anchors.fill: parent
        onClicked: {
          content.show = !content.show
        }
      }
    }

    Rectangle {
      id: content
      property bool show: true
      width: parent.width
      height: show ? tmpText.height : 0
      clip: true
      color: darkGrey

      Text {
        leftPadding: 20
        id: tmpText
        text: textFromModel(model)
        color: Material.theme == Material.Light ? "black" : "white"
        font.pointSize: 12
      }

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }
    }
  }
}
