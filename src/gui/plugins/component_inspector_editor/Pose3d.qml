/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspectorEditor"
import "qrc:/qml"

// Item displaying 3D pose information.
Rectangle {
  height: header.height + content.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  Connections {
    target: Pose3dImpl
    onPoseChanged: {
      // Update the spinner to show to correct values when the pose component
      // has changed, such as when a new entity is selected.
      xSpin.numberValue = model.data[0]
      ySpin.numberValue = model.data[1]
      zSpin.numberValue = model.data[2]
      rollSpin.numberValue = model.data[3]
      pitchSpin.numberValue = model.data[4]
      yawSpin.numberValue = model.data[5]
    }
  }

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Maximum spinbox value
  property double spinMax: Number.MAX_VALUE

  property int iconWidth: 20
  property int iconHeight: 20

  // Send new pose to C++
  function sendPose(_value) {
    Pose3dImpl.PoseUpdate(
      xSpin.numberValue,
      ySpin.numberValue,
      zSpin.numberValue,
      rollSpin.numberValue,
      pitchSpin.numberValue,
      yawSpin.numberValue
    );
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  Column {
    anchors.fill: parent

    // The expanding header. Make sure that the content to expand has an id set
    // to the value "content".
    ExpandingTypeHeader {
      id: header
      // Using the default header text values.
    }

    // Content
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? grid.height : 0
      clip: true
      color: "transparent"

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }


      GridLayout {
        id: grid
        width: parent.width
        columns: 6

        // Left spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }


        Component {
          id: plotIcon
          Image {
            property string componentInfo: ""
            source: "plottable_icon.svg"
            anchors.top: parent.top
            anchors.left: parent.left

            Drag.mimeData: { "text/plain" : (model === null) ? "" :
            "Component," + model.entity + "," + model.typeId + "," +
                           model.dataType + "," + componentInfo + "," + model.shortName
            }
            Drag.dragType: Drag.Automatic
            Drag.supportedActions : Qt.CopyAction
            Drag.active: dragMouse.drag.active
            // a point to drag from
            Drag.hotSpot.x: 0
            Drag.hotSpot.y: y
            MouseArea {
              id: dragMouse
              anchors.fill: parent
              drag.target: (model === null) ? null : parent
              onPressed: parent.grabToImage(function(result) {parent.Drag.imageSource = result.url })
              onReleased: parent.Drag.drop();
              cursorShape: Qt.DragCopyCursor
            }
        }
      }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: xText.width + indentation*3
          Loader {
            id: loaderX
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderX.item.componentInfo = "x"

          Text {
            id : xText
            text: ' X (m)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        StateAwareSpin {
          id: xSpin
          Layout.fillWidth: true
          height: 40
          numberValue: model.data[0]
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          Component.onCompleted: {
            xSpin.onChange.connect(sendPose)
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: rollText.width + indentation*3
          Loader {
            id: loaderRoll
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderRoll.item.componentInfo = "roll"

          Text {
            id: rollText
            text: ' Roll (rad)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        StateAwareSpin {
          id: rollSpin
          Layout.fillWidth: true
          height: 40
          numberValue: model.data[3]
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          Component.onCompleted: {
            rollSpin.onChange.connect(sendPose)
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }


        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: yText.width + indentation*3
          Loader {
            id: loaderY
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderY.item.componentInfo = "y"
          Text {
            id: yText
            text: ' Y (m)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        StateAwareSpin {
          id: ySpin
          Layout.fillWidth: true
          height: 40
          numberValue: model.data[1]
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          Component.onCompleted: {
            ySpin.onChange.connect(sendPose)
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: pitchText.width + indentation*3
          Loader {
            id: loaderPitch
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderPitch.item.componentInfo = "pitch";
          Text {
            id: pitchText
            text: ' Pitch (rad)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        StateAwareSpin {
          id: pitchSpin
          Layout.fillWidth: true
          height: 40
          numberValue: model.data[4]
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          Component.onCompleted: {
            pitchSpin.onChange.connect(sendPose)
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: zText.width + indentation*3
          Loader {
            id: loaderZ
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderZ.item.componentInfo = "z";
          Text {
            id: zText
            text: ' Z (m)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        StateAwareSpin {
          id: zSpin
          Layout.fillWidth: true
          height: 40
          numberValue: model.data[2]
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          Component.onCompleted: {
            zSpin.onChange.connect(sendPose)
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: yawText.width + indentation*3
          Loader {
            id: loaderYaw
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderYaw.item.componentInfo = "yaw";
          Text {
            id: yawText
            text: ' Yaw (rad)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        StateAwareSpin {
          id: yawSpin
          Layout.fillWidth: true
          height: 40
          numberValue: model.data[5]
          minValue: -Number.MAX_VALUE
          maxValue: Number.MAX_VALUE
          stepValue: 0.1
          Component.onCompleted: {
            yawSpin.onChange.connect(sendPose)
          }
        }
      }
    }
  }
}
