import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  background: Rectangle {
    color: "transparent"
  }
  ColumnLayout{
    CheckBox {
      id: showgrid
      text: qsTr("Show/Hide Grid")
      checked: true
      onClicked: {
        console.log("Checkbox state: " + checked)
      }
    }

    RowLayout{
      Label {
        text: "Vertical Cell Count: "
      }
      SpinBox {
        id: verticalCellCount
        from: 1
        to: 100
        value: 1
        editable: true
        onValueModified: {
          console.log("Vertical cell count: " + verticalCellCount.value)
        }
      }
    }

    RowLayout {
      Label {
        text: "Horizontal Cell Count: "
      }
      SpinBox {
        id: horizontalCellCount
        from: 1
        to: 100
        value: 20
        editable: true
        onValueModified: {
          console.log("Horizontal cell count: " + horizontalCellCount.value)
        }
      }
    }

    RowLayout {
      Label {
        text: "Cell Length (/m): "
      }

      DoubleSpinBox {
        id: cellLength
        from: 1
        value: 100
        onValueModified: {
          console.log("Cell length: " + cellLength.realValue)
        }
      }
    }

    RowLayout {
      ColumnLayout {
        Label {
          text: "Cartesian Pose"
        }

        Label {
          text: "X (/m): "
        }

        DoubleSpinBox {
          id: x
          value: 0
          onValueModified: {
            console.log("x: " + x.realValue)
          }
        }

        Label {
          text: "Y (/m): "
        }

        DoubleSpinBox {
          id: y
          value: 0
          onValueModified: {
            console.log("y: " + y.realValue)
          }
        }

        Label {
          text: "Z (/m): "
        }

        DoubleSpinBox {
          id: z
          value: 0
          onValueModified: {
            console.log("z: " + z.realValue)
          }
        }
      }

      ColumnLayout {
        Label {
          text: "Principal Pose"
        }

        Label {
          text: "Roll (/rad): "
        }

        DoubleSpinBox {
          id: roll
          from: 0
          to: 628*100
          value: 0
          onValueModified: {
            console.log("roll: " + roll.realValue)
          }
        }

        Label {
          text: "Pitch (/rad): "
        }

        DoubleSpinBox {
          id: pitch
          from: 0
          to: 628*100
          value: 0
          onValueModified: {
            console.log("pitch: " + pitch.realValue)
          }
        }

        Label {
          text: "Yaw (/rad): "
        }

        DoubleSpinBox {
          id: yaw
          from: 0
          to: 628*100
          value: 0
          onValueModified: {
            console.log("yaw: " + yaw.realValue)
          }
        }
      }
    }

    RowLayout {
      Label {
        text: "R: "
      }
      DoubleSpinBox {
        id: r
        from: 0
        to: 100*100
        value: 0
        onValueModified: {
          console.log("r: " + r.realValue)
        }
      }

      Label {
        text: "G: "
      }
      DoubleSpinBox {
        id: g
        from: 0
        to: 100*100
        value: 0
        onValueModified: {
          console.log("g: " + g.realValue)
        }
      }
    }

    RowLayout{
      Label {
        text: "B: "
      }
      DoubleSpinBox {
        id: b
        from: 0
        to: 100*100
        value: 0
        onValueModified: {
          console.log("b: " + b.realValue)
        }
      }

      Label {
        text: "A: "
      }
      DoubleSpinBox {
        id: a
        from: 0
        to: 100*100
        value: 0
        onValueModified: {
          console.log("a: " + a.realValue)
        }
      }
    }

    Button {
      id: color
      text: qsTr("Custom Color")
      onClicked: colorDialog.open()
    }
    ColorDialog {
      id: colorDialog
      title: "Please choose a color"
      visible: false
      onAccepted: {
          console.log("You chose: " + colorDialog.color)
          colorDialog.close()
      }
      onRejected: {
          console.log("Canceled")
          colorDialog.close()
      }
    }
  }
}