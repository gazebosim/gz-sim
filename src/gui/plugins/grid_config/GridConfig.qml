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
    }

    RowLayout{
      Label {
        text: "Vertical Cell Count"
      }
      SpinBox {
        id: verticalCellCount
        from: 0
        to: 100
      }
    }

    RowLayout {
      Label {
        text: "Horizontal Cell Count"
      }
      SpinBox {
        id: horizontalCellCount
        from: 0
        to: 100
      }
    }

    RowLayout {
      Label {
        text: "Cell Length (/m)"
      }

      DoubleSpinBox {
        id: cellLength
      }
    }

    RowLayout {
      ColumnLayout {
        Label {
          text: "Cartesian Pose"
        }

        Label {
          text: "x (/m)"
        }

        DoubleSpinBox {
          id: x
        }

        Label {
          text: "y (/m)"
        }

        DoubleSpinBox {
          id: y
        }

        Label {
          text: "z (/m)"
        }

        DoubleSpinBox {
          id: z
        }
      }

      ColumnLayout {
        Label {
          text: "Principal Pose"
        }

        Label {
          text: "roll (/rad)"
        }

        DoubleSpinBox {
          id: roll
        }

        Label {
          text: "pitch (/rad)"
        }

        DoubleSpinBox {
          id: pitch
        }

        Label {
          text: "yaw (/rad)"
        }

        DoubleSpinBox {
          id: yaw
        }
      }
    }

    RowLayout {
      Button {
        id: color
        text: qsTr("Select Grid Color")
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
}