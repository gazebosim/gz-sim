/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: root

  required property string resourceName
  required property string category
  property string metaText: ""
  property bool loading: false

  readonly property bool isDark: Material.theme === Material.Dark

  function categoryInfo(cat) {
    switch (cat) {
      case "drone":
        return { code: "UAV", label: "DRONE", hue: 0.56 }
      case "robot":
        return { code: "BOT", label: "ROBOT", hue: 0.60 }
      case "vehicle":
        return { code: "VEH", label: "VEHICLE", hue: 0.08 }
      case "sensor":
        return { code: "SEN", label: "SENSOR", hue: 0.76 }
      case "nature":
        return { code: "NAT", label: "NATURE", hue: 0.34 }
      case "building":
        return { code: "BLD", label: "BUILDING", hue: 0.12 }
      case "furniture":
        return { code: "FUR", label: "FURNITURE", hue: 0.06 }
      case "light":
        return { code: "LGT", label: "LIGHT", hue: 0.14 }
      case "prop":
        return { code: "PRP", label: "PROP", hue: 0.48 }
      default: {
        // Deterministic hue from resource name so generic models vary visually
        var hash = 0
        for (var i = 0; i < resourceName.length; ++i)
          hash = ((hash << 5) - hash) + resourceName.charCodeAt(i)
        var hue = (Math.abs(hash) % 360) / 360.0
        var trimmed = resourceName.trim()
        var parts = trimmed.split(/[\s_\-]+/)
        var initials = ""
        if (parts.length >= 2 && parts[0].length > 0 && parts[1].length > 0)
          initials = (parts[0][0] + parts[1][0]).toUpperCase()
        else if (trimmed.length >= 2)
          initials = trimmed.substring(0, 2).toUpperCase()
        else if (trimmed.length === 1)
          initials = trimmed.toUpperCase()
        else
          initials = "3D"
        return { code: initials, label: "MODEL", hue: hue }
      }
    }
  }

  readonly property var info: categoryInfo(category)
  readonly property color accentTint: Qt.hsla(
    info.hue, isDark ? 0.38 : 0.48, isDark ? 0.52 : 0.44, 1.0)
  readonly property color surfaceTint: Qt.hsla(
    info.hue, isDark ? 0.22 : 0.30, isDark ? 0.22 : 0.93, 1.0)

  radius: 4
  color: loading ?
    (isDark ? Material.color(Material.Grey, Material.Shade800) :
              Material.color(Material.Grey, Material.Shade200)) :
    surfaceTint
  clip: true

  // Subtle shimmer while a remote thumbnail is loading
  Rectangle {
    anchors.fill: parent
    visible: root.loading
    radius: root.radius
    color: root.isDark ? "#22ffffff" : "#22000000"

    SequentialAnimation on opacity {
      running: root.loading && root.visible
      loops: Animation.Infinite
      NumberAnimation { from: 0.25; to: 0.75; duration: 700; easing.type: Easing.InOutSine }
      NumberAnimation { from: 0.75; to: 0.25; duration: 700; easing.type: Easing.InOutSine }
    }
  }

  ColumnLayout {
    anchors.centerIn: parent
    width: parent.width - 12
    spacing: 4
    visible: !root.loading

    Rectangle {
      Layout.alignment: Qt.AlignHCenter
      width: 40
      height: 40
      radius: 20
      color: root.accentTint

      Label {
        anchors.centerIn: parent
        text: root.info.code
        color: "white"
        font.bold: true
        font.pointSize: 11
      }
    }

    RowLayout {
      Layout.alignment: Qt.AlignHCenter
      spacing: 4

      Rectangle {
        radius: 3
        color: root.isDark ? "#33ffffff" : "#18000000"
        implicitWidth: catLabel.implicitWidth + 10
        implicitHeight: catLabel.implicitHeight + 4

        Label {
          id: catLabel
          anchors.centerIn: parent
          text: root.info.label
          font.pointSize: 8
          font.bold: true
          opacity: 0.85
        }
      }

      Rectangle {
        visible: root.metaText !== ""
        radius: 3
        color: root.isDark ? "#26ffffff" : "#12000000"
        implicitWidth: metaLabel.implicitWidth + 10
        implicitHeight: metaLabel.implicitHeight + 4

        Label {
          id: metaLabel
          anchors.centerIn: parent
          text: root.metaText
          font.pointSize: 8
          opacity: 0.75
        }
      }
    }
  }
}
