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
import "qrc:/qml"

Rectangle {
  id: plot3D
  color: "transparent"
  Layout.minimumWidth: 400
  Layout.minimumHeight: 320
  anchors.fill: parent

  /**
   * Light grey according to theme
   */
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Dark grey according to theme
   */
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  Connections {
    target: Plot3D
    onLockedChanged: {
      lockButton.checked = Plot3D.Locked()
    }
  }

  Rectangle {
    id: header
    height: lockButton.height
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    width: parent.width
    color: darkGrey

    RowLayout {
      anchors.fill: parent
      spacing: 0

      Label {
        text: Plot3D.targetName.empty ? "No entity selected" : Plot3D.targetName
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 3
      }

      Item {
        height: entityLabel.height
        Layout.fillWidth: true
      }

      ToolButton {
        id: lockButton
        checkable: true
        checked: false
        text: "Lock entity"
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: lockButton.checked ? "qrc:/Gazebo/images/lock.svg" :
                                       "qrc:/Gazebo/images/unlock.svg"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        ToolTip.text: lockButton.checked ? "Unlock target selection"
            : "Lock target selection"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onToggled: {
          Plot3D.locked = lockButton.checked
        }
      }

      Label {
        id: entityLabel
        text: 'Entity ' + Plot3D.targetEntity
        Layout.minimumWidth: 80
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 5
      }
    }
  }
  ColumnLayout {
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right

    GridLayout {
      Layout.fillWidth: true
      columns: 6

      Text {
        text: "Offset"
        Layout.fillWidth: true
        Layout.row: 0
        Layout.column: 0
        Layout.columnSpan: 6
        color: "dimgrey"
        font.bold: true
        leftPadding: 5
      }

      Text {
        text: "X (m)"
        color: "dimgrey"
        Layout.row: 1
        Layout.column: 0
        leftPadding: 5
      }
      GzSpinBox {
        id: x
        Layout.fillWidth: true
        Layout.row: 1
        Layout.column: 1
        value: Plot3D.offset.x
        maximumValue: 1000000
        minimumValue: -1000000
        decimals: 2
        stepSize: 0.01
        onEditingFinished: Plot3D.SetOffset(Qt.vector3d(x.value, y.value, z.value))
      }
      Text {
        text: "Y (m)"
        color: "dimgrey"
        Layout.row: 1
        Layout.column: 2
        leftPadding: 5
      }
      GzSpinBox {
        id: y
        Layout.fillWidth: true
        value: Plot3D.offset.y
        maximumValue: 1000000
        minimumValue: -1000000
        decimals: 2
        stepSize: 0.01
        onEditingFinished: Plot3D.SetOffset(Qt.vector3d(x.value, y.value, z.value))
      }
      Text {
        text: "Z (m)"
        color: "dimgrey"
        Layout.row: 1
        Layout.column: 4
        leftPadding: 5
      }
      GzSpinBox {
        id: z
        Layout.fillWidth: true
        Layout.row: 1
        Layout.column: 5
        value: Plot3D.offset.z
        maximumValue: 1000000
        minimumValue: -1000000
        decimals: 2
        stepSize: 0.01
        onEditingFinished: Plot3D.SetOffset(Qt.vector3d(x.value, y.value, z.value))
      }

      Text {
        text: "Color"
        Layout.fillWidth: true
        Layout.row: 2
        Layout.column: 0
        Layout.columnSpan: 6
        color: "dimgrey"
        font.bold: true
        leftPadding: 5
      }

      Text {
        text: "R"
        color: "dimgrey"
        Layout.row: 3
        Layout.column: 0
        leftPadding: 5
      }

      GzSpinBox {
        id: r
        Layout.fillWidth: true
        Layout.row: 3
        Layout.column: 1
        value: Plot3D.color.x
        maximumValue: 1.00
        minimumValue: 0.00
        decimals: 2
        stepSize: 0.01
        onEditingFinished: Plot3D.SetColor(Qt.vector3d(r.value, g.value, b.value))
      }

      Text {
        text: "G"
        Layout.row: 3
        Layout.column: 2
        color: "dimgrey"
        leftPadding: 5
      }

      GzSpinBox {
        id: g
        Layout.fillWidth: true
        Layout.row: 3
        Layout.column: 3
        value: Plot3D.color.y
        maximumValue: 1.00
        minimumValue: 0.00
        decimals: 2
        stepSize: 0.01
        onEditingFinished: Plot3D.SetColor(Qt.vector3d(r.value, g.value, b.value))
      }

      Text {
        text: "B"
        Layout.row: 3
        Layout.column: 4
        color: "dimgrey"
        leftPadding: 5
      }

      GzSpinBox {
        id: b
        Layout.fillWidth: true
        Layout.row: 3
        Layout.column: 5
        value: Plot3D.color.z
        maximumValue: 1.00
        minimumValue: 0.00
        decimals: 2
        stepSize: 0.01
        onEditingFinished: Plot3D.SetColor(Qt.vector3d(r.value, g.value, b.value))
      }
    }

    GridLayout {
      Layout.fillWidth: true
      columns: 2

      Text {
        text: "Min distance between points (m)"
        color: "dimgrey"
        Layout.row: 0
        Layout.column: 0
        leftPadding: 5
      }
      GzSpinBox {
        id: minDist
        Layout.fillWidth: true
        Layout.row: 0
        Layout.column: 1
        value: Plot3D.minDistance
        maximumValue: 1000000
        minimumValue: 0
        decimals: 6
        stepSize: 0.01
        onEditingFinished: Plot3D.SetMinDistance(minDist.value)
      }

      Text {
        text: "Max points"
        color: "dimgrey"
        Layout.row: 1
        Layout.column: 0
        leftPadding: 5
      }
      GzSpinBox {
        id: maxPoints
        Layout.fillWidth: true
        Layout.row: 1
        Layout.column: 1
        value: Plot3D.maxPoints
        maximumValue: 1000000
        minimumValue: 0
        decimals: 0
        stepSize: 100
        onEditingFinished: Plot3D.SetMaxPoints(maxPoints.value)
      }
    }

    // Bottom spacer
    Item {
      width: 10
      Layout.row: 6
      Layout.column: 0
      Layout.columnSpan: 6
      Layout.fillHeight: true
    }
  }
}
