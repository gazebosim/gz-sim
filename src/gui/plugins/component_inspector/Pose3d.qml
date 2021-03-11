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
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying 3D pose information.
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

  // Read-only / write
  property bool readOnly: {
    var isModel = entityType == "model"
    return !(isModel) || nestedModel
  }

  // Loaded item for X
  property var xItem: {}

  // Loaded item for Y
  property var yItem: {}

  // Loaded item for Z
  property var zItem: {}

  // Loaded item for roll
  property var rollItem: {}

  // Loaded item for pitch
  property var pitchItem: {}

  // Loaded item for yaw
  property var yawItem: {}

  // Send new pose to C++
  function sendPose() {
    // TODO(anyone) There's a loss of precision when these values get to C++
    componentInspector.onPose(
      xItem.value,
      yItem.value,
      zItem.value,
      rollItem.value,
      pitchItem.value,
      yawItem.value
    );
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  /**
   * Used to create a spin box
   */
  Component {
    id: writableNumber
    IgnSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: -spinMax
      maximumValue: spinMax
      decimals: 6
      onEditingFinished: {
        sendPose()
      }
    }
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

        Text {
          text: 'X (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: xLoader
            anchors.fill: parent
            property double numberValue: model.data[0]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
            onLoaded: {
              xItem = xLoader.item
            }
          }
        }

        Text {
          text: 'Roll (rad)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: rollLoader
            anchors.fill: parent
            property double numberValue: model.data[3]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
            onLoaded: {
              rollItem = rollLoader.item
            }
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin
        }

        Text {
          text: 'Y (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: yLoader
            anchors.fill: parent
            property double numberValue: model.data[1]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
            onLoaded: {
              yItem = yLoader.item
            }
          }
        }

        Text {
          text: 'Pitch (rad)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: pitchLoader
            anchors.fill: parent
            property double numberValue: model.data[4]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
            onLoaded: {
              pitchItem = pitchLoader.item
            }
          }
        }

        Text {
          text: 'Z (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: zLoader
            anchors.fill: parent
            property double numberValue: model.data[2]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
            onLoaded: {
              zItem = zLoader.item
            }
          }
        }

        Text {
          text: 'Yaw (rad)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: yawLoader
            anchors.fill: parent
            property double numberValue: model.data[5]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
            onLoaded: {
              yawItem = yawLoader.item
            }
          }
        }
      }
    }
  }
}
