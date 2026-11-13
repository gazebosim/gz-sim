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
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.15

// Sibling components. Qt 5 does not find them in qrc without this. Qualified,
// because a type named PerformanceOverlay would hide the plugin object, which
// gz-gui 8 publishes to QML under that same name.
import "qrc:/PerformanceOverlay" as Parts

Rectangle {
  id: overlay

  readonly property bool dark: Material.theme === Material.Dark
  readonly property color physicsColor: dark ? "#CC7F18" : "#C76A00"
  readonly property color renderColor: dark ? "#1FA596" : "#00917C"
  readonly property color otherColor: dark ? "#8B95A8" : "#8A93A3"
  readonly property color cpuColor: dark ? "#5F8BF0" : "#2F5FD0"
  readonly property color gpuColor: dark ? "#E0559A" : "#C4317C"
  readonly property color bothColor: dark ? "#9D72E0" : "#7B46B4"
  readonly property color goodColor: dark ? "#4CB782" : "#1E8E5A"
  readonly property color textOnFill: dark ? "#0C1118" : "#FFFFFF"
  readonly property color trackColor: Qt.rgba(Material.foreground.r,
      Material.foreground.g, Material.foreground.b, 0.12)
  readonly property color axisColor: Qt.rgba(Material.foreground.r,
      Material.foreground.g, Material.foreground.b, 0.35)
  readonly property color mutedColor: Qt.rgba(Material.foreground.r,
      Material.foreground.g, Material.foreground.b, 0.7)

  readonly property var report: PerformanceOverlay.report
  readonly property bool hasReport: report.headline !== undefined
  readonly property real idle: !hasReport ? 0 : Math.max(0, 1 - report.physics -
      report.renderWait - report.otherSystems - report.loop)

  function locationColor(location) {
    if (location === "physics")
      return physicsColor
    if (location === "rendering")
      return renderColor
    return otherColor
  }

  function verdictColor() {
    if (!report.limited)
      return goodColor
    if (report.resource === "cpu")
      return cpuColor
    if (report.resource === "gpu")
      return gpuColor
    if (report.resource === "cpu_and_gpu")
      return bothColor
    return otherColor
  }

  function largest(rows, key) {
    return rows.reduce(function(top, row) {
      return Math.max(top, row[key])
    }, 0)
  }

  function formatMs(share) {
    return (1000 * share * report.windowSeconds /
        Math.max(1, report.iterations)).toFixed(2) + " ms/step"
  }

  anchors.fill: parent
  Layout.minimumWidth: 340
  // A floating card leaves its title bar out of its height, and parent.y is
  // how far the card pushes its content down to make room for that bar.
  Layout.minimumHeight: content.implicitHeight + 2 * content.anchors.margins +
      (parent ? parent.y : 0)
  color: Qt.rgba(Material.background.r, Material.background.g,
      Material.background.b, 0.92)

  ColumnLayout {
    id: content
    anchors.fill: parent
    anchors.margins: 12
    spacing: 12

    // The verdict
    RowLayout {
      spacing: 10
      visible: overlay.hasReport
      Layout.fillWidth: true

      Rectangle {
        color: overlay.verdictColor()
        radius: 7
        Layout.preferredWidth: 14
        Layout.preferredHeight: 14
        Layout.alignment: Qt.AlignTop
        Layout.topMargin: 4

        Label {
          anchors.centerIn: parent
          visible: !overlay.report.limited
          text: "✓"
          color: overlay.textOnFill
          font.pixelSize: 10
          font.bold: true
        }
      }

      ColumnLayout {
        spacing: 2
        Layout.fillWidth: true

        Label {
          text: overlay.report.headline || ""
          font.pixelSize: 16
          font.bold: true
          wrapMode: Text.Wrap
          Layout.fillWidth: true
        }

        Label {
          text: overlay.report.detail || ""
          visible: text !== ""
          color: overlay.mutedColor
          font.pixelSize: 12
          wrapMode: Text.Wrap
          maximumLineCount: 3
          elide: Text.ElideRight
          Layout.fillWidth: true
        }
      }

      ColumnLayout {
        spacing: 0
        Layout.alignment: Qt.AlignTop

        Label {
          text: overlay.hasReport ? overlay.report.rtf.toFixed(2) : ""
          font.pixelSize: 24
          font.weight: Font.DemiBold
          Layout.alignment: Qt.AlignRight
        }

        Label {
          text: !overlay.hasReport ? "" : overlay.report.desiredRtf > 0 ?
              "RTF of " + overlay.report.desiredRtf.toFixed(2) : "RTF, no target"
          color: overlay.mutedColor
          font.pixelSize: 11
          Layout.alignment: Qt.AlignRight
        }
      }
    }

    Label {
      text: PerformanceOverlay.notice
      visible: text !== ""
      color: overlay.mutedColor
      font.pixelSize: 12
      wrapMode: Text.Wrap
      Layout.fillWidth: true
    }

    // Real time factor over time
    ColumnLayout {
      spacing: 4
      visible: overlay.hasReport
      Layout.fillWidth: true

      Label {
        text: "RTF over the last reports"
        color: overlay.mutedColor
        font.pixelSize: 11
      }

      Parts.RtfHistory {
        samples: PerformanceOverlay.history
        target: overlay.hasReport ? overlay.report.desiredRtf : 0
        targetColor: overlay.axisColor
        colorOf: function(sample) {
          return sample.limited ? overlay.locationColor(sample.location) :
              overlay.trackColor
        }
        Layout.fillWidth: true
        Layout.preferredHeight: implicitHeight
      }
    }

    // Where the time of a step goes
    ColumnLayout {
      spacing: 4
      visible: overlay.hasReport
      Layout.fillWidth: true

      Label {
        text: "Where the time goes"
        color: overlay.mutedColor
        font.pixelSize: 11
      }

      Parts.StackedBar {
        showValues: true
        trackColor: overlay.trackColor
        textColor: overlay.textOnFill
        parts: !overlay.hasReport ? [] : [
          {label: "Physics", value: overlay.report.physics,
           color: overlay.physicsColor},
          {label: "Rendering", value: overlay.report.renderWait,
           color: overlay.renderColor},
          {label: "Other systems and the loop",
           value: overlay.report.otherSystems + overlay.report.loop,
           color: overlay.otherColor}
        ]
        Layout.fillWidth: true
        Layout.preferredHeight: implicitHeight
      }

      Row {
        spacing: 12

        Repeater {
          model: [
            {label: "Physics", swatch: overlay.physicsColor},
            {label: "Rendering", swatch: overlay.renderColor},
            {label: "Other", swatch: overlay.otherColor},
            {label: "Idle", swatch: overlay.trackColor}
          ]

          Row {
            required property var modelData
            spacing: 4

            Rectangle {
              anchors.verticalCenter: parent.verticalCenter
              width: 8
              height: 8
              radius: 4
              color: parent.modelData.swatch
            }

            Label {
              text: parent.modelData.label
              color: overlay.mutedColor
              font.pixelSize: 11
            }
          }
        }
      }
    }

    // CPU or GPU, for the render thread
    ColumnLayout {
      spacing: 4
      visible: overlay.hasReport && overlay.report.showRenderThread
      Layout.fillWidth: true

      RowLayout {
        spacing: 12
        Layout.fillWidth: true

        Label {
          text: !overlay.hasReport ? "" : overlay.report.softwareRenderer ?
              "Render thread: software rasterizer" :
              "Render thread: " + overlay.report.renderFrames + " frames"
          color: overlay.mutedColor
          font.pixelSize: 11
          elide: Text.ElideRight
          Layout.fillWidth: true
        }

        Repeater {
          model: [
            {label: "GPU wait", swatch: overlay.gpuColor},
            {label: "CPU", swatch: overlay.cpuColor}
          ]

          Row {
            required property var modelData
            spacing: 4

            Rectangle {
              anchors.verticalCenter: parent.verticalCenter
              width: 8
              height: 8
              radius: 4
              color: parent.modelData.swatch
            }

            Label {
              text: parent.modelData.label
              color: overlay.mutedColor
              font.pixelSize: 11
            }
          }
        }
      }

      Item {
        Layout.fillWidth: true
        Layout.preferredHeight: 24

        Parts.StackedBar {
          id: renderBar
          anchors.fill: parent
          showValues: true
          trackColor: overlay.trackColor
          textColor: overlay.textOnFill
          parts: !overlay.hasReport ? [] : [
            {label: "Waiting for the GPU",
             value: overlay.report.gpuWaitRatio, color: overlay.gpuColor},
            {label: "Working on the CPU",
             value: 1 - overlay.report.gpuWaitRatio, color: overlay.cpuColor}
          ]
        }

        // Limits of the verdict: a GPU wait below 35% means the CPU limits
        // the rendering, and above 65% means the GPU does.
        Repeater {
          model: [0.35, 0.65]

          Rectangle {
            required property real modelData
            x: renderBar.width * modelData - 1
            width: 2
            height: parent.height
            color: Material.background
          }
        }
      }

      Item {
        implicitHeight: cpuZone.implicitHeight
        Layout.fillWidth: true

        Label {
          id: cpuZone
          text: "CPU limits"
          color: overlay.mutedColor
          font.pixelSize: 11
        }

        Label {
          anchors.horizontalCenter: parent.horizontalCenter
          text: "both"
          color: overlay.mutedColor
          font.pixelSize: 11
        }

        Label {
          anchors.right: parent.right
          text: "GPU limits"
          color: overlay.mutedColor
          font.pixelSize: 11
        }
      }
    }

    // The sensors
    ColumnLayout {
      spacing: 2
      visible: overlay.hasReport && overlay.report.sensors.length > 0
      Layout.fillWidth: true

      Label {
        text: "Sensors"
        color: overlay.mutedColor
        font.pixelSize: 11
      }

      Repeater {
        model: overlay.hasReport ? overlay.report.sensors : []

        Parts.LoadRow {
          required property var modelData

          name: modelData.label
          tooltip: modelData.name + "\nGPU wait " +
              Math.round(100 * modelData.gpuShare) + "%, CPU " +
              Math.round(100 * (modelData.share - modelData.gpuShare)) + "%"
          value: modelData.share
          whole: overlay.largest(overlay.report.sensors, "share")
          clickable: modelData.model !== ""
          trackColor: overlay.trackColor
          parts: [
            {label: "GPU wait", value: modelData.gpuShare,
             color: overlay.gpuColor},
            {label: "CPU", value: modelData.share - modelData.gpuShare,
             color: overlay.cpuColor}
          ]
          onClicked: PerformanceOverlay.SelectModel(modelData.model)
          Layout.fillWidth: true
        }
      }
    }

    // The sensors that produced no data
    ColumnLayout {
      spacing: 2
      visible: overlay.hasReport && overlay.report.dataless.length > 0
      Layout.fillWidth: true

      Label {
        text: "Sensors without data"
        color: overlay.mutedColor
        font.pixelSize: 11

        HoverHandler {
          id: datalessHover
        }
        ToolTip.visible: datalessHover.hovered
        ToolTip.delay: 300
        ToolTip.text: "Sensors only work while something subscribes to " +
            "their data.\nSubscribe like your application does to measure " +
            "their cost."
      }

      Repeater {
        model: overlay.hasReport ? overlay.report.dataless : []

        Label {
          required property var modelData

          text: modelData.reason + ": " + modelData.sensors
          wrapMode: Text.Wrap
          maximumLineCount: 2
          elide: Text.ElideRight
          font.pixelSize: 12
          Layout.fillWidth: true

          HoverHandler {
            id: groupHover
          }
          ToolTip.visible: groupHover.hovered
          ToolTip.delay: 300
          ToolTip.text: modelData.reason + ":\n" + modelData.tooltip
        }
      }
    }

    // The models
    ColumnLayout {
      spacing: 2
      visible: overlay.hasReport && overlay.report.models.length > 0
      Layout.fillWidth: true

      Label {
        text: "Models (~ estimated)"
        color: overlay.mutedColor
        font.pixelSize: 11
      }

      Repeater {
        model: overlay.hasReport ? overlay.report.models : []

        Parts.LoadRow {
          required property var modelData

          name: modelData.name
          prefix: modelData.physics + modelData.scene > modelData.total / 2 ?
              "~" : ""
          tooltip: modelData.name + "\nPhysics ~" +
              overlay.formatMs(modelData.physics) + "\nScene ~" +
              overlay.formatMs(modelData.scene) + "\nSensors " +
              overlay.formatMs(modelData.sensors) + "\nPlugins " +
              overlay.formatMs(modelData.plugins)
          value: modelData.total
          whole: overlay.largest(overlay.report.models, "total")
          clickable: true
          trackColor: overlay.trackColor
          parts: [
            {label: "Physics (estimated)", value: modelData.physics,
             color: overlay.physicsColor},
            {label: "Scene (estimated)", value: modelData.scene,
             color: overlay.renderColor},
            {label: "Sensors", value: modelData.sensors,
             color: overlay.renderColor},
            {label: "Plugins", value: modelData.plugins,
             color: overlay.otherColor}
          ]
          onClicked: PerformanceOverlay.SelectModel(modelData.name)
          Layout.fillWidth: true
        }
      }
    }

    // The systems
    ColumnLayout {
      spacing: 2
      visible: overlay.hasReport && overlay.report.systems.length > 0
      Layout.fillWidth: true

      Label {
        text: "Systems"
        color: overlay.mutedColor
        font.pixelSize: 11
      }

      Repeater {
        model: overlay.hasReport ? overlay.report.systems : []

        Parts.LoadRow {
          required property var modelData

          name: modelData.name
          value: modelData.share
          whole: overlay.largest(overlay.report.systems, "share")
          trackColor: overlay.trackColor
          parts: [
            {label: modelData.name, value: modelData.share,
             color: overlay.otherColor}
          ]
          Layout.fillWidth: true
        }
      }
    }

    RowLayout {
      spacing: 8
      Layout.fillWidth: true

      ToolButton {
        text: PerformanceOverlay.monitoring ? "Stop" : "Start"
        font.pixelSize: 11
        Layout.preferredHeight: 28
        onClicked: PerformanceOverlay.SetMonitoring(
            !PerformanceOverlay.monitoring)
      }

      Label {
        text: PerformanceOverlay.paused ?
            "Paused, showing the last report" : !overlay.hasReport ? "" :
            overlay.report.windowSeconds.toFixed(1) + " s report, " +
            overlay.report.iterations + " steps"
        horizontalAlignment: Text.AlignRight
        color: overlay.mutedColor
        font.pixelSize: 11
        elide: Text.ElideLeft
        Layout.fillWidth: true
      }
    }
  }
}
