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
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import "qrc:/qml"

Rectangle {
  id: gridconfigwindow
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 660

  Column {
    spacing: 5
    anchors.fill: parent
    anchors.leftMargin: 20

    CheckBox {
      id: showgrid
      text: qsTr("Show/Hide Grid")
      checked: true
      onClicked: {
        GridConfig.OnShow(checked)
      }
    }

    Text {
      text: "Cell Count"
      color: "dimgrey"
      font.bold: true
    }

    Row {
      spacing: 25

      Column {
        spacing: 2
        Text {
          id: vercelltext
          color: "dimgrey"
          text: "Vertical"
        }

        IgnSpinBox {
          id: verticalCellCount
          maximumValue: 1000
          minimumValue: 0
          value: 0
          onEditingFinished: GridConfig.UpdateVCellCount(verticalCellCount.value)
        }
      }

      Column {
        spacing: 2
        Text {
          id: honcelltext
          color: "dimgrey"
          text: "Horizontal"
        }

        IgnSpinBox {
          id: horizontalCellCount
          maximumValue: 1000
          minimumValue: 1
          value: 20
          onEditingFinished: GridConfig.UpdateHCellCount(horizontalCellCount.value)
        }
      }
    }

    Row {
      Column {
        spacing: 2
        Text {
          id: celllengthtext
          text: "Cell Length"
          color: "dimgrey"
          font.bold: true
        }

        IgnSpinBox {
          id: cellLength
          maximumValue: 1000.00
          minimumValue: 0.01
          value: 1.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.UpdateCellLength(cellLength.value)
        }
      }
    }

    Row {
      spacing: 25

      Column {
        spacing: 2
        Text {
          id: cartesian
          color: "dimgrey"
          font.bold: true
          text: "Position (/m)"
        }

        Text {
          text: "X"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: x
          value: 0.00
          maximumValue: 1000.00
          minimumValue: -1000.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
        }

        Text {
          text: "Y"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: y
          value: 0.00
          maximumValue: 1000.00
          minimumValue: -1000.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
        }

        Text {
          text: "Z"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: z
          value: 0.00
          maximumValue: 1000.00
          minimumValue: -1000.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
        }
      }

      Column {
        spacing: 2
        Text {
          id: principal
          text: "Rotation (/rad)"
          color: "dimgrey"
          font.bold: true
        }

        Text {
          text: "Roll"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: roll
          maximumValue: 6.28
          minimumValue: 0.00
          value: 0.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
        }

        Text {
          text: "Pitch"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: pitch
          maximumValue: 6.28
          minimumValue: 0.00
          value: 0.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
        }

        Text {
          text: "Yaw"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: yaw
          maximumValue: 6.28
          minimumValue: 0.00
          value: 0.00
          decimals: 2
          stepSize: 0.01
          onEditingFinished: GridConfig.SetPose(x.value, y.value, z.value, roll.value, pitch.value, yaw.value)
        }
      }
    }

    Text {
      text: "Color"
      color: "dimgrey"
      font.bold: true
    }

    Row {
      spacing: 25

      Column {
        spacing: 2
        Text {
          text: "R"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: r
          maximumValue: 1.00
          minimumValue: 0.00
          value: 0.7
          stepSize: 0.01
          decimals: 2
          onEditingFinished: GridConfig.SetColor(r.value, g.value, b.value, a.value)
        }

        Text {
          text: "G"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: g
          maximumValue: 1.00
          minimumValue: 0.00
          value: 0.7
          stepSize: 0.01
          decimals: 2
          onEditingFinished: GridConfig.SetColor(r.value, g.value, b.value, a.value)
        }
      }

      Column {
        spacing: 2
        Text {
          text: "B"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: b
          maximumValue: 1.00
          minimumValue: 0.00
          value: 0.7
          stepSize: 0.01
          decimals: 2
          onEditingFinished: GridConfig.SetColor(r.value, g.value, b.value, a.value)
        }
        Text {
          text: "A"
          color: "dimgrey"
        }

        IgnSpinBox {
          id: a
          maximumValue: 1.00
          minimumValue: 0.00
          value: 1.0
          stepSize: 0.01
          decimals: 2
          onEditingFinished: GridConfig.SetColor(r.value, g.value, b.value, a.value)
        }
      }
    }

    Button {
      id: color
      text: qsTr("Custom Color")
      onClicked: colorDialog.open()

      ColorDialog {
        id: colorDialog
        title: "Choose a grid color"
        visible: false
        onAccepted: {
          r.value = colorDialog.color.r
          g.value = colorDialog.color.g
          b.value = colorDialog.color.b
          a.value = colorDialog.color.a
          GridConfig.SetColor(colorDialog.color.r, colorDialog.color.g, colorDialog.color.b, colorDialog.color.a)
          colorDialog.close()
        }
        onRejected: {
          colorDialog.close()
        }
      }
    }
  }
}
