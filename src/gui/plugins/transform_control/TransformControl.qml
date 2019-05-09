import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {

  background: Rectangle {
    implicitHeight: 24

    Rectangle {
        width: parent.width
        height: 1
        anchors.bottom: parent.bottom
    }
  }

  ButtonGroup {
    buttons: rowLayout.children
  }

  RowLayout {
    id: rowLayout
    ToolButton {
      text: "A"
      checkable: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "arrow.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        TransformControl.OnMode("select")
      }
    }
    ToolButton{
      text: "T"
      checkable: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "translate.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        TransformControl.OnMode("translate")
      }
    }
    ToolButton {
      text: "R"
      checkable: true
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "rotate.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        TransformControl.OnMode("rotate")
      }
    }
    // TODO(anyone) enable scale button when supported is added in ign-physics
    // ToolButton {
    //   text: "S"
    //   checkable: true
    //   contentItem: Image {
    //     fillMode: Image.Pad
    //     horizontalAlignment: Image.AlignHCenter
    //     verticalAlignment: Image.AlignVCenter
    //     source: "scale.png"
    //     sourceSize.width: 24;
    //     sourceSize.height: 24;
    //   }
    //   onClicked: {
    //     TransformControl.OnMode("scale")
    //   }
    // }
  }
}
