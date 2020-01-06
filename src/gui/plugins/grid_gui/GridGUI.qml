import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls 1.4 as OldCtrl
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3


Rectangle {
  id: gridconfig
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375

  property int tooltipDelay: 500
  property int tooltipTimeout: 1000

  Column {
    anchors.fill: parent
    anchors.margins: 10

    Row {
      CheckBox {
        text: qsTr("Show/Hide Grid")
        checked: true
      }
    }
    
    Row {
      Label {
        text: "Vertical Cell Count"
      }
      SpinBox {
        id: verticalCellCount
        from: 0
        to: 100
      }
      Label {
        text: "Horizontal Cell Count"
      }
      SpinBox {
        id: horizontalCellCount
        from: 0
        to: 100
      }
    }

    Row {
      Label {
        text: "Vertical Cell Length (/m)"
      }

      DoubleSpinBox {
        id: verticalCellLength
      }

      Label {
        text: "Horizontal Cell Length (/m)"
      }

      DoubleSpinBox {
        id: horizontalCellLength
      }
    }
    
    Row {
      Column {
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
      Row {
        Column {
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
    }

    Row {
      Button {
        id: color
        text: qsTr("Select Grid Color")
        onClicked: model.submit()
      }
    }
   
    ColorDialog {
      id: colorDialog
      title: "Please choose a color"
      onAccepted: {
          console.log("You chose: " + colorDialog.color)
          Qt.quit()
      }
      onRejected: {
          console.log("Canceled")
          Qt.quit()
      }
      Component.onCompleted: visible = true
    }
  }
}


