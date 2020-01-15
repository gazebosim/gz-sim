import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import "qrc:/qml"

Rectangle {
  id: gridconfigwindow
  color: "transparent"
  Layout.minimumWidth: 370
  Layout.minimumHeight: 650

  Column {
    anchors.fill: parent
    anchors.margins: 10

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
      Column {
        Text {
          id: vercelltext
          color: "dimgrey"
          text: "Vertical"
        }

        SpinBox {
          id: verticalCellCount
          from: 0
          to: 100
          value: 0
          editable: true
          scale: 0.8
          onValueModified: GridConfig.UpdateVerCellCount(verticalCellCount.value)
        }
      }
      
      Column {
        Text {
          id: honcelltext
          color: "dimgrey"
          text: "Horizontal"
        }

        SpinBox {
          id: horizontalCellCount
          from: 1
          to: 100
          value: 20
          editable: true
          scale: 0.8
          onValueModified: GridConfig.UpdateHonCellCount(horizontalCellCount.value)
        }
      }
    }
    
    Row {
      Column {
        Text {
          id: celllengthtext
          text: "Cell Length"
          color: "dimgrey"
          font.bold: true
        }

        IgnDoubleSpinBox {
          id: cellLength
          from: 1
          value: 100
          scale: 0.8
          onValueModified: GridConfig.UpdateCellLength(cellLength.realValue)
        }
      }
    }

    Row {
      Column {
        Text {
          id: cartesian
          color: "dimgrey"
          font.bold: true
          text: "Cartesian Pose (/m)"
        }

        Text {
          text: "X"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: x
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Text {
          text: "Y"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: y
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Text {
          text: "Z"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: z
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }
      }
      
      Column {
        Text {
          id: principal
          text: "Principal Pose (/rad)"
          color: "dimgrey"
          font.bold: true
        }

        Text {
          text: "Roll"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: roll
          from: 0
          to: 628*100
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Text {
          text: "Pitch"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: pitch
          from: 0
          to: 628*100
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Text {
          text: "Yaw"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: yaw
          from: 0
          to: 628*100
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }
      }      
    }

    Text {
      text: "Color"
      color: "dimgrey"
      font.bold: true
    }

    Row {
      Column {
        Text {
          text: "R"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: r
          from: 0
          to: 100*100
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
        }

        Text {
          text: "G"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: g
          from: 0
          to: 100*100
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
        }
      }

      Column {
        Text {
          text: "B"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: b
          from: 0
          to: 100*100
          value: 0
          scale: 0.8
          onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
        }
        Text {
          text: "A"
          color: "dimgrey"
        }

        IgnDoubleSpinBox {
          id: a
          from: 0
          to: 100*100
          value: 100
          scale: 0.8
          onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
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