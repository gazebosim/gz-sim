import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  // TODO(anyone) enable scale button when support is added in ign-physics
  // function activateScale() {
  //   scale.checked = true;
  //   TransformControl.OnMode("scale");
  // } 
 
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
          if (mouse.button === Qt.RightButton) {
            snapTranslateMenu.open()
          }
        }
        Menu {
          id: snapTranslateMenu
          Text {
            id: x
            text: qsTr("X (0-100) :")
          }
          TextField {
            id: xEntry
            placeholderText: qsTr("Meters")
            validator: DoubleValidator {
              bottom: 0
              top: 100
              decimals: 10
            }
            onEditingFinished: {
              TransformControl.OnSnapUpdate(
                xEntry.text, yEntry.text, zEntry.text,
                rollEntry.text, pitchEntry.text, yawEntry.text,
                xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
              )
            }
          }
          Text {
            id: y
            text: qsTr("Y (0-100) :")
          }
          TextField {
            id: yEntry
            placeholderText: qsTr("Meters")
            validator: DoubleValidator {
              bottom: 0
              top: 100
              decimals: 10
            }
            onEditingFinished: {
              TransformControl.OnSnapUpdate(
                xEntry.text, yEntry.text, zEntry.text,
                rollEntry.text, pitchEntry.text, yawEntry.text,
                xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
              )
            }
          }
          Text {
            id: z
            text: qsTr("Z (0-100) :")
          }
          TextField {
            id: zEntry
            placeholderText: qsTr("Meters")
            validator: DoubleValidator {
              bottom: 0
              top: 100
              decimals: 10
            }
            onEditingFinished: {
              TransformControl.OnSnapUpdate(
                xEntry.text, yEntry.text, zEntry.text,
                rollEntry.text, pitchEntry.text, yawEntry.text,
                xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
              )
            }
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
          if (mouse.button === Qt.RightButton) {
            snapRotateMenu.open()
          }
        }
        Menu {
          id: snapRotateMenu
          Text {
            id: roll
            text: qsTr("R (0-90) :")
          }
          TextField {
            id: rollEntry
            placeholderText: qsTr("Degrees")
            validator: DoubleValidator {
              bottom: 0
              top: 90
              decimals: 10
            }
            onEditingFinished: {
              TransformControl.OnSnapUpdate(
                xEntry.text, yEntry.text, zEntry.text,
                rollEntry.text, pitchEntry.text, yawEntry.text,
                xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
              )
            }
          }
          Text {
            id: pitch
            text: qsTr("P (0-90) :")
          }
          TextField {
            id: pitchEntry
            placeholderText: qsTr("Degrees")
            validator: DoubleValidator {
              bottom: 0
              top: 90
              decimals: 10
            }
            onEditingFinished: {
              TransformControl.OnSnapUpdate(
                xEntry.text, yEntry.text, zEntry.text,
                rollEntry.text, pitchEntry.text, yawEntry.text,
                xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
              )
            }
          }
          Text {
            id: yaw
            text: qsTr("Y (0-90) :")
          }
          TextField {
            id: yawEntry
            placeholderText: qsTr("Degrees")
            validator: DoubleValidator {
              bottom: 0
              top: 90
              decimals: 10
            }
            onEditingFinished: {
              TransformControl.OnSnapUpdate(
                xEntry.text, yEntry.text, zEntry.text,
                rollEntry.text, pitchEntry.text, yawEntry.text,
                xScaleEntry.text, yScaleEntry.text, zScaleEntry.text
              )
            }
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
    Text {
      id: xScaleEntry
    }
    Text {
      id: yScaleEntry
    }
    Text {
      id: zScaleEntry
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
    // 
    //    readonly property bool square: scale.contentItem.width <= scale.contentItem.height
    // 
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
