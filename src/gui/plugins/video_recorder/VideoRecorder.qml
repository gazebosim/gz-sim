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
import QtCore
import QtQuick 2.9
import QtQuick.Controls
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3

import QtQuick.Dialogs

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
        fileMode: FileDialog.SaveFile
        currentFolder: StandardPaths.writableLocation(StandardPaths.HomeLocation)
        property var fileFormat: ""
        property var selectedFormat: ""

        function getFormat(url) {
          return (url.slice(url.lastIndexOf(".") + 1))
        }

        onAccepted: {
          fileFormat = getFormat(selectedFile.toString())
          if (fileFormat == selectedFile.toString()) {
            // no format specified
            _VideoRecorder.OnSave(fileFormat + "." + selectedFormat)
            close()
          }
          else if (fileFormat != selectedFormat){
            // wrong format specified
            mismatchDialog.open()
          } else {
            // correct format specified
            _VideoRecorder.OnSave(selectedFile)
            close()
          }
        }
        onRejected: {
          _VideoRecorder.OnCancel()
          close()
        }
    }

    Dialog {
      id: mismatchDialog
      title: "Enconding and filename mismatch"
      modal: true
      focus: false
      width: 700
      height: 200
      parent: Overlay.overlay
      x: (parent.width - width) / 2
      y: (parent.height - height) / 2
      standardButtons: Dialog.Save | Dialog.Abort

      GridLayout {
        id: grid
        rows: 2
        flow: GridLayout.TopToBottom
        anchors.fill: parent

        Label {
          text: "File extension doesn't correspond with video encoding."
          font.pixelSize: 20
        }

        Label {
          text: "Click 'Abort' to change the name or 'Save' to continue."
          font.pixelSize: 20
        }
      }

      onAccepted: {
        _VideoRecorder.OnSave(fileDialog.selectedFile)
        _VideoRecorder.close()
        close()
      }
      onRejected: {
        fileDialog.open()
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
          fileDialog.nameFilters = ["*.mp4"]
          fileDialog.selectedFormat = "mp4"
          _VideoRecorder.OnStart("mp4")
          animation.start()
        }
      }
      MenuItem {
        text: "ogv"
        onTriggered: {
          fileDialog.nameFilters = ["*.ogv"]
          fileDialog.selectedFormat = "ogv"
          _VideoRecorder.OnStart("ogv")
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
          _VideoRecorder.OnStop()
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
