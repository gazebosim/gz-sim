import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
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
    // TODO(anyone) enable scale button when supported is added in ign-physics
    // ToolButton {
    //   text: "S"
    //   checkable: true
    //   contentItem: Image {
    //     fillMode: Image.Pad
    //     horizontalAlignment: Image.AlignHCenter
    //     verticalAlignment: Image.AlignVCenter
    //     source: "scale.png"
    //     sourceSize.width: 24;
    //     sourceSize.height: 24;
    //   }
    //  // Almost an exact copy from upstream, adding `checked`
    //  background: Ripple {
    //    implicitWidth: 48
    //    implicitHeight: 48

    //    readonly property bool square: select.contentItem.width <= select.contentItem.height

    //    x: (parent.width - width) / 2
    //    y: (parent.height - height) / 2
    //    clip: !square
    //    width: square ? parent.height / 2 : parent.width
    //    height: square ? parent.height / 2 : parent.height
    //    pressed: select.pressed
    //    anchor: select
    //    active: select.enabled && (select.down || select.visualFocus || select.hovered || select.checked)
    //    color: select.Material.rippleColor
    //  }
    //   onClicked: {
    //     TransformControl.OnMode("scale")
    //   }
    // }
  }
}
