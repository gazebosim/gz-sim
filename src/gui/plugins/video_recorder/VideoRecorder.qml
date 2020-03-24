/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.0

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  background: Rectangle {
    color: "transparent"
  }

  RowLayout {

    FileDialog {
        id: fileDialog
        title: "Save the recorded video"
        folder: shortcuts.home
        selectExisting: false
        onAccepted: {
          VideoRecorder.OnSave(fileDialog.fileUrl)
          close()
        }
        onRejected: {
          VideoRecorder.OnCancel()
          close()
        }
    }

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
      id: recordMenu
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

    Menu {
      id: stopMenu
      y: record.height
      MenuItem {
        text: "Stop"
        onTriggered: {
          animation.stop()
          VideoRecorder.OnStop()
          fileDialog.open()
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
        animation.running ? stopMenu.open() :  recordMenu.open()
      }
    }
  }
}
