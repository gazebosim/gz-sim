import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/qml"

ToolBar {
  id: transformControl
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  // TODO(anyone) enable scale button when support is added in ign-physics
  // function activateScale() {
  //   scale.checked = true;
  //   TransformControl.OnMode("scale");
  // } 

  property color snapTitle: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)
  
  property color snapItem: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  function activateTranslate() {
    translate.checked = true;
    TransformControl.OnMode("translate");
  }

  function activateRotate() {
    rotate.checked = true;
    TransformControl.OnMode("rotate");
  }
  
  function activateSelect() {
    select.checked = true;
    TransformControl.OnMode("select");
  }

  function windowWidth() {
    return transformControl.Window.window ? (transformControl.Window.window.width) : 0
  }

  function windowHeight() {
    return transformControl.Window.window ? (transformControl.Window.window.height) : 0
  }

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  // TODO(anyone) enable scale button when support is added in ign-physics
  // Shortcut {
  //   sequence: "S"
  //   onActivated: activateScale()
  // }

  Shortcut {
    sequence: "T"
    onActivated: activateTranslate()
  }

  Shortcut {
    sequence: "R"
    onActivated: activateRotate()
  }
  
  Shortcut {
    sequence: "Esc"
    onActivated: activateSelect()
  }

  RowLayout {
    ToolButton {
      id: select
      checkable: true
      checked: true
      ButtonGroup.group: group
      ToolTip.text: "Select mode"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "arrow.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      // Almost an exact copy from upstream, adding `checked`
      background: Ripple {
        implicitWidth: 48
        implicitHeight: 48

        readonly property bool square: select.contentItem.width <= select.contentItem.height

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        clip: !square
        width: square ? parent.height / 2 : parent.width
        height: square ? parent.height / 2 : parent.height
        pressed: select.pressed
        anchor: select
        active: select.enabled && (select.down || select.visualFocus || select.hovered || select.checked)
        color: select.Material.rippleColor
      }
      onClicked: {
        TransformControl.OnMode("select")
      }
    }
    ToolButton{
      id: translate
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Translate mode"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "translate.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        onClicked: {
          if (mouse.button === Qt.LeftButton) {
            translate.checked = true;
            TransformControl.OnMode("translate")
          }
        }
      }
      // Almost an exact copy from upstream, adding `checked`
      background: Ripple {
        implicitWidth: 48
        implicitHeight: 48

        readonly property bool square: translate.contentItem.width <= translate.contentItem.height

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        clip: !square
        width: square ? parent.height / 2 : parent.width
        height: square ? parent.height / 2 : parent.height
        pressed: translate.pressed
        anchor: translate
        active: translate.enabled && (translate.down || translate.visualFocus || translate.hovered || translate.checked)
        color: translate.Material.rippleColor
      }
    }
    ToolButton {
      id: rotate
      text: "R"
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Rotate mode"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "rotate.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        onClicked: {
          if (mouse.button === Qt.LeftButton) {
            rotate.checked = true;
            TransformControl.OnMode("rotate")
          }
        }
      }
      // Almost an exact copy from upstream, adding `checked`
      background: Ripple {
        implicitWidth: 48
        implicitHeight: 48

        readonly property bool square: rotate.contentItem.width <= rotate.contentItem.height

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        clip: !square
        width: square ? parent.height / 2 : parent.width
        height: square ? parent.height / 2 : parent.height
        pressed: rotate.pressed
        anchor: rotate
        active: rotate.enabled && (rotate.down || rotate.visualFocus || rotate.hovered || rotate.checked)
        color: rotate.Material.rippleColor
      }
    }
    ToolButton {
      id: snap
      text: "S"
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Enter custom snap values"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "magnet.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        onClicked: {
          if (mouse.button === Qt.LeftButton) {
            snapDialog.open()
          }
        }
      }
    }
    Dialog {
      id: snapDialog
      parent: transformControl.Window.window ? transformControl.Window.window.contentItem : transformControl
      x: (windowWidth() - width) / 2
      y: (windowHeight() - height) / 2
      width: 330
      height: 250
      modal: true
      focus: true
      title: "Snap values"
      GridLayout {
        columns: 6
        columnSpacing: 30
        Text {
          text: "Translation (m)"
          font.weight: Font.Bold
          Layout.columnSpan: 2
          Layout.row: 0
          Layout.column: 0
          bottomPadding: 10
        }
        Text {
          text: "X"
          Layout.row: 1
          Layout.column: 0
        }
        IgnSpinBox {
          id: xEntry
          minimumValue: 0.01
          maximumValue: 100.0
          decimals: 2
          stepSize: 0.01
          value: 1
          Layout.row: 1
          Layout.column: 1
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              0, 0, 0
            )
          }
        }
        Text {
          text: "Y"
          Layout.row: 2
          Layout.column: 0
        }
        IgnSpinBox {
          id: yEntry
          minimumValue: 0.01
          maximumValue: 100.0
          decimals: 2
          stepSize: 0.01
          value: 1
          Layout.row: 2
          Layout.column: 1
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              0, 0, 0
            )
          }
        }
        Text {
          text: "Z"
          Layout.row: 3
          Layout.column: 0
        }
        IgnSpinBox {
          id: zEntry
          minimumValue: 0.01
          maximumValue: 100.0
          decimals: 2
          stepSize: 0.01
          value: 1
          Layout.row: 3
          Layout.column: 1
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              0, 0, 0
            )
          }
        }
        Text {
          text: "Rotation (deg)"
          font.weight: Font.Bold
          Layout.columnSpan: 2
          Layout.row: 0
          Layout.column: 2
          bottomPadding: 10
        }
        Text {
          text: "Roll"
          Layout.row: 1
          Layout.column: 2
        }
        IgnSpinBox {
          id: rollEntry
          minimumValue: 0.01
          maximumValue: 180.0
          decimals: 2
          stepSize: 0.01
          value: 45
          Layout.row: 1
          Layout.column: 3
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              0, 0, 0
            )
          }
        }
        Text {
          text: "Pitch"
          Layout.row: 2
          Layout.column: 2
        }
        IgnSpinBox {
          id: pitchEntry
          minimumValue: 0.01
          maximumValue: 180.0
          decimals: 2
          stepSize: 0.01
          value: 45
          Layout.row: 2
          Layout.column: 3
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              0, 0, 0
            )
          }
        }
        Text {
          text: "Yaw"
          Layout.row: 3
          Layout.column: 2
        }
        IgnSpinBox {
          id: yawEntry
          minimumValue: 0.01
          maximumValue: 180.0
          decimals: 2
          stepSize: 0.01
          value: 45
          Layout.row: 3
          Layout.column: 3
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              0, 0, 0
            )
          }
        }

        // TODO(anyone) enable scale snap values below when support is added in ign-physics
        // Also be sure to replace the above placeholder 0's, in the `OnSnapUpdate` call to
        // xScaleEntry.value, yScaleEntry.value, and zScaleEntry.value, respectively
        /*
        Text {
          text: "Scaling"
          font.weight: Font.Bold
          Layout.columnSpan: 2
          Layout.row: 0
          Layout.column: 4
          bottomPadding: 10
        }
        Text {
          text: "X"
          Layout.row: 1
          Layout.column: 4
        }
        IgnSpinBox {
          id: xScaleEntry
          minimumValue: 0.01
          maximumValue: 180.0
          decimals: 2
          stepSize: 0.01
          value: 45
          Layout.row: 1
          Layout.column: 5
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              xScaleEntry.value, yScaleEntry.value, zScaleEntry.value
            )
          }
        }
        Text {
          text: "Y"
          Layout.row: 2
          Layout.column: 4
        }
        IgnSpinBox {
          id: yScaleEntry
          minimumValue: 0.01
          maximumValue: 180.0
          decimals: 2
          stepSize: 0.01
          value: 45
          Layout.row: 2
          Layout.column: 5
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              xScaleEntry.value, yScaleEntry.value, zScaleEntry.value
            )
          }
        }
        Text {
          text: "Z"
          Layout.row: 3
          Layout.column: 4
        }
        IgnSpinBox {
          id: zScaleEntry
          minimumValue: 0.01
          maximumValue: 180.0
          decimals: 2
          stepSize: 0.01
          value: 45
          Layout.row: 3
          Layout.column: 5
          onEditingFinished: {
            TransformControl.OnSnapUpdate(
              xEntry.value, yEntry.value, zEntry.value,
              rollEntry.value, pitchEntry.value, yawEntry.value,
              xScaleEntry.value, yScaleEntry.value, zScaleEntry.value
            )
          }
        }
        */
      }
    }
    
    // TODO(anyone) enable scale button when support is added in ign-physics
    // ToolButton {
    //   id: scale
    //   text: "S"
    //   checkable: true
    //   ButtonGroup.group: group
    //   ToolTip.text: "Scale mode"
    //   ToolTip.visible: hovered
    //   ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
    //   contentItem: Image {
    //     fillMode: Image.Pad
    //     horizontalAlignment: Image.AlignHCenter
    //     verticalAlignment: Image.AlignVCenter
    //     source: "scale.png"
    //     sourceSize.width: 24;
    //     sourceSize.height: 24;
    //   }
    //   MouseArea {
    //     anchors.fill: parent
    //     acceptedButtons: Qt.LeftButton | Qt.RightButton
    //     onClicked: {
    //       if (mouse.button === Qt.LeftButton) {
    //         scale.checked = true;
    //         TransformControl.OnMode("scale")
    //       }
    //       if (mouse.button === Qt.RightButton) {
    //         snapScaleMenu.open()
    //       }
    //     }
    //     Menu {
    //       id: snapScaleMenu
    //       Text {
    //         id: xScale
    //         text: qsTr("X (0-5) :")
    //       }
    //       TextField {
    //         id: xScaleEntry
    //         placeholderText: qsTr("Size")
    //         validator: DoubleValidator {
    //           bottom: 0
    //           top: 5
    //           decimals: 10
    //         }
    //        onEditingFinished: {
    //           TransformControl.OnSnapUpdate(
    //             xEntry.text, yEntry.text, zEntry.text,
    //             rollEntry.text, pitchEntry.text, yawEntry.text,
    //             xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
    //           )
    //         }
    //       }
    //       Text {
    //         id: yScale
    //         text: qsTr("Y (0-5) :")
    //       }
    //       TextField {
    //         id: yScaleEntry
    //         placeholderText: qsTr("Size")
    //         validator: DoubleValidator {
    //           bottom: 0
    //           top: 5
    //           decimals: 10
    //         }
    //        onEditingFinished: {
    //           TransformControl.OnSnapUpdate(
    //             xEntry.text, yEntry.text, zEntry.text,
    //             rollEntry.text, pitchEntry.text, yawEntry.text,
    //             xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
    //           )
    //         }
    //       }
    //       Text {
    //         id: zScale
    //         text: qsTr("Z (0-5) :")
    //       }
    //       TextField {
    //         id: zScaleEntry
    //         placeholderText: qsTr("Size")
    //         validator: DoubleValidator {
    //           bottom: 0
    //           top: 5
    //           decimals: 10
    //         }
    //        onEditingFinished: {
    //           TransformControl.OnSnapUpdate(
    //             xEntry.text, yEntry.text, zEntry.text,
    //             rollEntry.text, pitchEntry.text, yawEntry.text,
    //             xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
    //           )
    //         }
    //       }
    //     }
    //   }
    //  // Almost an exact copy from upstream, adding `checked`
    //  background: Ripple {
    //    implicitWidth: 48
    //    implicitHeight: 48

    //    readonly property bool square: scale.contentItem.width <= scale.contentItem.height

    //    x: (parent.width - width) / 2
    //    y: (parent.height - height) / 2
    //    clip: !square
    //    width: square ? parent.height / 2 : parent.width
    //    height: square ? parent.height / 2 : parent.height
    //    pressed: select.pressed
    //    anchor: scale
    //    active: scale.enabled && (scale.down || scale.visualFocus || scale.hovered || scale.checked)
    //    color: scale.Material.rippleColor
    //  }
    // }
  }
}
