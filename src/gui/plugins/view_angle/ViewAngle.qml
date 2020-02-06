import QtQuick 2.9
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  Layout.minimumWidth: 200
  Layout.minimumHeight: 200

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  GridLayout {
    columns: 4
    ToolButton {
      id: top
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the top"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 0
      Layout.column: 1
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_top.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, -1)
      }
    }
    ToolButton {
      id: home
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "Reset View Angle"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 0
      Layout.column: 3
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_home.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 0)
      }
    }
    ToolButton {
      id: left
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the left"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 0
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_left.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 1, 0)
      }
    }
    ToolButton {
      id: front
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the front"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 1
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_front.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(-1, 0, 0)
      }
    }
    ToolButton {
      id: right
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the right"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 2
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_right.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, -1, 0)
      }
    }
    ToolButton {
      id: back
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the back"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 1
      Layout.column: 3
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_back.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(1, 0, 0)
      }
    }
    ToolButton {
      id: bottom
      checkable: true
      ButtonGroup.group: group
      ToolTip.text: "View from the bottom"
      ToolTip.visible: hovered
      ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
      Layout.row: 2
      Layout.column: 1
      contentItem: Image {
        fillMode: Image.Pad
        horizontalAlignment: Image.AlignHCenter
        verticalAlignment: Image.AlignVCenter
        source: "view_angle_bottom.png"
        sourceSize.width: 24;
        sourceSize.height: 24;
      }
      onClicked: {
        ViewAngle.OnAngleMode(0, 0, 1)
      }
    }
  }
}
