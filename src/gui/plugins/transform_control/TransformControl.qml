/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

  // TODO(anyone) enable scale button when support is added in gz-physics
  // function activateScale() {
  //   scale.checked = true;
  //   TransformControl.OnMode("scale");
  // }

  property color snapTitle: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade900) :
    Material.color(Material.Grey, Material.Shade200)

  property color snapItem: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade800) :
    Material.color(Material.Grey, Material.Shade100)

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

  function updateSnapValues() {
    xEntry.value = TransformControl.xSnap();
    yEntry.value = TransformControl.ySnap();
    zEntry.value = TransformControl.zSnap();
    rollEntry.value = TransformControl.rollSnap();
    pitchEntry.value = TransformControl.pitchSnap();
    yawEntry.value = TransformControl.yawSnap();
    // TODO(anyone) enable scale button when support is added in gz-physics
    // xScaleEntry.value = TransformControl.xScaleSnap()
    // yScaleEntry.value = TransformControl.yScaleSnap()
    // zScaleEntry.value = TransformControl.zScaleSnap()
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

  Connections {
    target: TransformControl
    onNewSnapValues: updateSnapValues();
  }

  Connections {
    target: TransformControl
    onActivateSelect: activateSelect();
  }

  Connections {
    target: TransformControl
    onActivateTranslate: activateTranslate();
  }

  Connections {
    target: TransformControl
    onActivateRotate: activateRotate();
  }

  RowLayout {
    spacing: 2
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
      onClicked: {
        TransformControl.OnMode("translate")
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
      onClicked: {
        TransformControl.OnMode("rotate")
      }
    }
    // TODO(anyone) enable scale snap values below when support is added in gz-physics
    // Also be sure to replace the placeholder 0's in all of the `OnSnapUpdate` calls in
    // this file to xScaleEntry.value, yScaleEntry.value, and zScaleEntry.value, respectively
    /*
    ToolButton {
      id: scale
      text: "S"
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Scale mode"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "scale.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      // Almost an exact copy from upstream, adding `checked`
      background: Ripple {
        implicitWidth: 48
        implicitHeight: 48

        readonly property bool square: scale.contentItem.width <= scale.contentItem.height

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        clip: !square
        width: square ? parent.height / 2 : parent.width
        height: square ? parent.height / 2 : parent.height
        pressed: scale.pressed
        anchor: scale
        active: scale.enabled && (scale.down || scale.visualFocus || scale.hovered || scale.checked)
        color: scale.Material.rippleColor
      }
      onClicked: {
        TransformControl.OnMode("scale")
      }
    }
    */

    ToolButton {
      id: snap
      text: "N"
      checkable: false
      ToolTip.text: "Enter custom snap values"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "snap.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      // Almost an exact copy from upstream, adding `checked`
      background: Ripple {
        implicitWidth: 48
        implicitHeight: 48

        readonly property bool square: snap.contentItem.width <= snap.contentItem.height

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        clip: !square
        width: square ? parent.height / 2 : parent.width
        height: square ? parent.height / 2 : parent.height
        pressed: snap.pressed
        anchor: snap
        active: snap.enabled && (snap.down || snap.visualFocus || snap.hovered || snap.checked)
        color: snap.Material.rippleColor
      }
      onClicked: {
        snapDialog.open()
      }
    }
    ToolButton {
      id: gridSnap
      text: "N"
      checkable: false
      ToolTip.text: "Set snap values to grid"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "snap_to_grid.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        TransformControl.OnSnapToGrid()
      }
    }
    Dialog {
      id: snapDialog
      parent: transformControl.Window.window ? transformControl.Window.window.contentItem : transformControl
      x: (windowWidth() - width) / 2
      y: (windowHeight() - height) / 2
      width: 360
      height: 250
      modal: true
      focus: true
      title: "Snap values"
      GridLayout {
        columns: 6
        columnSpacing: 30
        Text {
          text: "Translation (m)"
          color: snapTitle
          font.weight: Font.Bold
          Layout.columnSpan: 2
          Layout.row: 0
          Layout.column: 0
          bottomPadding: 10
        }
        Text {
          text: "X"
          color: snapItem
          Layout.row: 1
          Layout.column: 0
        }
        GzSpinBox {
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
          color: snapItem
          Layout.row: 2
          Layout.column: 0
        }
        GzSpinBox {
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
          color: snapItem
          Layout.row: 3
          Layout.column: 0
        }
        GzSpinBox {
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
          color: snapTitle
          Layout.columnSpan: 2
          Layout.row: 0
          Layout.column: 2
          bottomPadding: 10
        }
        Text {
          text: "Roll"
          color: snapItem
          Layout.row: 1
          Layout.column: 2
        }
        GzSpinBox {
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
          color: snapItem
          Layout.row: 2
          Layout.column: 2
        }
        GzSpinBox {
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
          color: snapItem
          Layout.row: 3
          Layout.column: 2
        }
        GzSpinBox {
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
      }
    }
  }
}
