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
  Layout.minimumHeight: 375
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
        text: Plot3D.targetName
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
  GridLayout {
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    columns: 6

    Text {
      text: "Offset"
      Layout.fillWidth: true
      Layout.row: 0
      Layout.column: 0
      Layout.columnSpan: 6
      color: Material.Grey
      leftPadding: 5
    }

    Text {
      text: "X (m)"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 0
      leftPadding: 5
    }
    IgnSpinBox {
      id: x
      Layout.fillWidth: true
      Layout.row: 1
      Layout.column: 1
      value: 0
      maximumValue: 1000000
      minimumValue: -1000000
      decimals: 6
      stepSize: 0.01
      onEditingFinished: Plot3D.SetOffset(x.value, y.value, z.value)
    }
    Text {
      text: "Y (m)"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 2
      leftPadding: 5
    }
    IgnSpinBox {
      id: y
      Layout.fillWidth: true
      Layout.row: 1
      Layout.column: 3
      value: 0
      maximumValue: 1000000
      minimumValue: -1000000
      decimals: 6
      stepSize: 0.01
      onEditingFinished: Plot3D.SetOffset(x.value, y.value, z.value)
    }
    Text {
      text: "Z (m)"
      color: "dimgrey"
      Layout.row: 1
      Layout.column: 4
      leftPadding: 5
    }
    IgnSpinBox {
      id: z
      Layout.fillWidth: true
      Layout.row: 1
      Layout.column: 5
      value: 0
      maximumValue: 1000000
      minimumValue: -1000000
      decimals: 6
      stepSize: 0.01
      onEditingFinished: Plot3D.SetOffset(x.value, y.value, z.value)
    }

    Text {
      Layout.columnSpan: 6
      text: "Color"
      color: "dimgrey"
      font.bold: true
      Layout.row: 2
      Layout.column: 0
    }

    Text {
      text: "R"
      color: "dimgrey"
      Layout.row: 3
      Layout.column: 0
      leftPadding: 5
    }

    IgnSpinBox {
      id: r
      Layout.row: 3
      Layout.column: 1
      maximumValue: 1.00
      minimumValue: 0.00
      value: 0
      stepSize: 0.01
      decimals: 2
      onEditingFinished: Plot3D.SetColor(r.value, g.value, b.value)
    }

    Text {
      text: "G"
      Layout.row: 3
      Layout.column: 2
      color: "dimgrey"
      leftPadding: 5
    }

    IgnSpinBox {
      id: g
      Layout.row: 3
      Layout.column: 3
      maximumValue: 1.00
      minimumValue: 0.00
      value: 0
      stepSize: 0.01
      decimals: 2
      onEditingFinished: Plot3D.SetColor(r.value, g.value, b.value)
    }

    Text {
      text: "B"
      Layout.row: 3
      Layout.column: 4
      color: "dimgrey"
      leftPadding: 5
    }

    IgnSpinBox {
      id: b
      Layout.row: 3
      Layout.column: 5
      maximumValue: 1.00
      minimumValue: 0.00
      value: 1.0
      stepSize: 0.01
      decimals: 2
      onEditingFinished: Plot3D.SetColor(r.value, g.value, b.value)
    }

    Text {
      text: "Min distance between points (m)"
      color: "dimgrey"
      Layout.row: 4
      Layout.column: 0
      Layout.columnSpan: 3
      leftPadding: 5
    }
    IgnSpinBox {
      id: minDist
      Layout.fillWidth: true
      Layout.row: 4
      Layout.column: 3
      Layout.columnSpan: 3
      value: 0.05
      maximumValue: 1000000
      minimumValue: -1000000
      decimals: 6
      stepSize: 0.01
      onEditingFinished: Plot3D.SetMinDistance(minDist.value)
    }

    Text {
      text: "Max points"
      color: "dimgrey"
      Layout.row: 5
      Layout.column: 0
      Layout.columnSpan: 3
      leftPadding: 5
    }
    IgnSpinBox {
      id: maxPoints
      Layout.fillWidth: true
      Layout.row: 5
      Layout.column: 3
      Layout.columnSpan: 3
      value: 1000
      maximumValue: 1000000
      minimumValue: -1000000
      decimals: 0
      stepSize: 100
      onEditingFinished: Plot3D.SetMaxPoints(maxPoints.value)
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
