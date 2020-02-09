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
  color: "transparent"

  // Maximum spinbox value
  property double spinMax: 1000000

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
      height: show ? grid.height : 0
      clip: true
      color: darkGrey

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
          width: 5
        }

        Text {
          text: 'X (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        IgnSpinBox {
          id: xSpin
          value: model.data[0]
          minimumValue: -spinMax
          maximumValue: spinMax
          decimals: xSpin.width < 100 ? 2 : 6
          Layout.fillWidth: true
        }

        Text {
          text: 'Roll (rad)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        IgnSpinBox {
          id: rollSpin
          value: model.data[3]
          minimumValue: -spinMax
          maximumValue: spinMax
          decimals: rollSpin.width < 100 ? 2 : 6
          Layout.fillWidth: true
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: 5
        }

        Text {
          text: 'Y (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        IgnSpinBox {
          id: ySpin
          value: model.data[1]
          minimumValue: -spinMax
          maximumValue: spinMax
          decimals: ySpin.width < 100 ? 2 : 6
          Layout.fillWidth: true
        }

        Text {
          text: 'Pitch (rad)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        IgnSpinBox {
          id: pitchSpin
          value: model.data[4]
          minimumValue: -spinMax
          maximumValue: spinMax
          decimals: pitchSpin.width < 100 ? 2 : 6
          Layout.fillWidth: true
        }

        Text {
          text: 'Z (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        IgnSpinBox {
          id: zSpin
          value: model.data[2]
          minimumValue: -spinMax
          maximumValue: spinMax
          decimals: zSpin.width < 100 ? 2 : 6
          Layout.fillWidth: true
        }

        Text {
          text: 'Yaw (m)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
        }

        IgnSpinBox {
          id: yawSpin
          value: model.data[5]
          minimumValue: -spinMax
          maximumValue: spinMax
          decimals: yawSpin.width < 100 ? 2 : 6
          Layout.fillWidth: true
        }
      }
    }
  }
}
