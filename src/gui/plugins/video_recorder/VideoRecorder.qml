import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  background: Rectangle {
    color: "transparent"
  }

  RowLayout {

    SequentialAnimation {
      id: animation;
      loops: Animation.Infinite
      alwaysRunToEnd: true
      PropertyAnimation {
        target: record;
        property: "opacity"
        from: 1.0
        to: 0.1
        duration: 500
      }
      PropertyAnimation {
        target: record;
        property: "opacity"
        from: 0.1
        to: 1.0
        duration: 500
      }
    }

    Menu {
      id: menu
      y: record.height
      MenuItem {
        text: "mp4"
        onTriggered: {
          VideoRecorder.OnStart("mp4")
          animation.start()
        }
      }
      MenuItem {
        text: "ogv"
        onTriggered: {
          VideoRecorder.OnStart("ogv")
          animation.start()
        }
      }
    }

    ToolButton {
      id: record
      checkable: true
      ToolTip.text: animation.running ? "End Video and Save" : "Record Video"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "video_camera.svg"
        sourceSize.width: 24
        sourceSize.height: 24
      }

      background: Ripple {
        implicitWidth: 48
        implicitHeight: 48

        readonly property bool square: record.contentItem.width <= record.contentItem.height

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        clip: !square
        width: square ? parent.height / 2 : parent.width
        height: square ? parent.height / 2 : parent.height
        pressed: record.pressed
        anchor: record
        active: record.enabled && (record.down || record.visualFocus || record.hovered || record.checked)
        color: record.Material.rippleColor
      }
      onClicked: {
        animation.running ? stopRecording() :  menu.open()
      }

      function stopRecording(){
        animation.stop()
        VideoRecorder.OnStop()
      }
    }
  }
}
