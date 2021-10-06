/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying wrench information.
Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Maximum spinbox value
  property double spinMax: 1000000

  property int iconWidth: 20
  property int iconHeight: 20

  // Loaded item for X force
  property var xForceItem: {}

  // Loaded item for Y force
  property var yForceItem: {}

  // Loaded item for Z force
  property var zForceItem: {}

  // Loaded item for X torque
  property var xTorqueItem: {}

  // Loaded item for Y torque
  property var yTorqueItem: {}

  // Loaded item for Z torque
  property var zTorqueItem: {}

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  /**
   * Used to create a read-only number
   */
  Component {
    id: readOnlyNumber
    Text {
      id: numberText
      anchors.fill: parent
      horizontalAlignment: Text.AlignRight
      verticalAlignment: Text.AlignVCenter
      text: {
        var decimals = numberText.width < 100 ? 2 : 6
        return numberValue.toFixed(decimals)
      }
    }
  }

  Column {
    anchors.fill: parent

    // Header
    Rectangle {
      id: header
      width: parent.width
      height: typeHeader.height
      color: "transparent"

      RowLayout {
        anchors.fill: parent
        Item {
          width: margin
        }
        Image {
          id: icon
          sourceSize.height: indentation
          sourceSize.width: indentation
          fillMode: Image.Pad
          Layout.alignment : Qt.AlignVCenter
          source: content.show ?
              "qrc:/Gazebo/images/minus.png" : "qrc:/Gazebo/images/plus.png"
        }
        TypeHeader {
          id: typeHeader
        }
        Item {
          Layout.fillWidth: true
        }
      }
      MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: {
          content.show = !content.show
        }
        onEntered: {
          header.color = highlightColor
        }
        onExited: {
          header.color = "transparent"
        }
      }
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
          Layout.preferredWidth: xForceText.width + indentation*3
          Loader {
            id: loaderXForce
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderXForce.item.componentInfo = "x"

          Text {
            id : xForceText
            text: ' Force X (N)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
      }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: xForceLoader
            anchors.fill: parent
            property double numberValue: model.data[0]
            sourceComponent: readOnlyNumber
            onLoaded: {
              xForceItem = xForceLoader.item
            }
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: rollText.width + indentation*3
          Loader {
            id: loaderXTorque
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderXTorque.item.componentInfo = "roll"

          Text {
            id: rollText
            text: ' Torque X (Nm)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: xTorqueLoader
            anchors.fill: parent
            property double numberValue: model.data[3]
            sourceComponent: readOnlyNumber
            onLoaded: {
              xTorqueItem = xTorqueLoader.item
            }
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
          Layout.preferredWidth: yForceText.width + indentation*3
          Loader {
            id: loaderYForce
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderYForce.item.componentInfo = "y"
          Text {
            id: yForceText
            text: ' Force Y (N)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: yForceLoader
            anchors.fill: parent
            property double numberValue: model.data[1]
            sourceComponent: readOnlyNumber
            onLoaded: {
              yForceItem = yForceLoader.item
            }
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: yTorqueText.width + indentation*3
          Loader {
            id: loaderYTorque
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderYTorque.item.componentInfo = "yTorque";
          Text {
            id: yTorqueText
            text: ' Torque Y (Nm)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: yTorqueLoader
            anchors.fill: parent
            property double numberValue: model.data[4]
            sourceComponent: readOnlyNumber
            onLoaded: {
              yTorqueItem = yTorqueLoader.item
            }
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: zForceText.width + indentation*3
          Loader {
            id: loaderZForce
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderZForce.item.componentInfo = "z";
          Text {
            id: zForceText
            text: ' Force Z (N)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: zForceLoader
            anchors.fill: parent
            property double numberValue: model.data[2]
            sourceComponent: readOnlyNumber
            onLoaded: {
              zForceItem = zForceLoader.item
            }
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: zTorqueText.width + indentation*3
          Loader {
            id: loaderZTorque
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderZTorque.item.componentInfo = "zTorque";
          Text {
            id: zTorqueText
            text: ' Torque Z (Nm)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: zTorqueLoader
            anchors.fill: parent
            property double numberValue: model.data[5]
            sourceComponent: readOnlyNumber
            onLoaded: {
              zTorqueItem = zTorqueLoader.item
            }
          }
        }
      }
    }
  }
}
