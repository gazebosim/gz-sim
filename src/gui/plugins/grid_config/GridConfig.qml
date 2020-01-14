import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import "qrc:/qml"

ToolBar {
  Layout.minimumWidth: 300
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
        GridConfig.OnShow(checked)
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
        onValueModified: GridConfig.UpdateVerCellCount(verticalCellCount.value)
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
        onValueModified: GridConfig.UpdateHonCellCount(horizontalCellCount.value)
      }
    }

    RowLayout {
      Label {
        text: "Cell Length (/m): "
      }

      IgnDoubleSpinBox {
        id: cellLength
        from: 1
        value: 100
        onValueModified: GridConfig.UpdateCellLength(cellLength.realValue)
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

        IgnDoubleSpinBox {
          id: x
          value: 0
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Label {
          text: "Y (/m): "
        }

        IgnDoubleSpinBox {
          id: y
          value: 0
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)        }

        Label {
          text: "Z (/m): "
        }

        IgnDoubleSpinBox {
          id: z
          value: 0
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }
      }

      ColumnLayout {
        Label {
          text: "Principal Pose"
        }

        Label {
          text: "Roll (/rad): "
        }

        IgnDoubleSpinBox {
          id: roll
          from: 0
          to: 628*100
          value: 0
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Label {
          text: "Pitch (/rad): "
        }

        IgnDoubleSpinBox {
          id: pitch
          from: 0
          to: 628*100
          value: 0
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }

        Label {
          text: "Yaw (/rad): "
        }

        IgnDoubleSpinBox {
          id: yaw
          from: 0
          to: 628*100
          value: 0
          onValueModified: GridConfig.SetPose(x.realValue, y.realValue, z.realValue, roll.realValue, pitch.realValue, yaw.realValue)
        }
      }
    }

    RowLayout {
      Label {
        text: "R: "
      }
      IgnDoubleSpinBox {
        id: r
        from: 0
        to: 100*100
        value: 0
        onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
      }

      Label {
        text: "G: "
      }
      IgnDoubleSpinBox {
        id: g
        from: 0
        to: 100*100
        value: 0
        onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
      }
    }

    RowLayout{
      Label {
        text: "B: "
      }
      IgnDoubleSpinBox {
        id: b
        from: 0
        to: 100*100
        value: 0
        onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
      }

      Label {
        text: "A: "
      }
      IgnDoubleSpinBox {
        id: a
        from: 0
        to: 100*100
        value: 100
        onValueModified: GridConfig.SetColor(r.realValue, g.realValue, b.realValue, a.realValue)
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
        GridConfig.SetColor(colorDialog.color)
        colorDialog.close()
      }
      onRejected: {
        colorDialog.close()
      }
    }
  }
}