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

Item {
  id: card

  required property string resourceName
  required property string ownerName
  required property string sdfPath
  required property string thumbnailUrl
  required property string category
  required property string formattedFileSize
  required property bool isFuel
  required property bool isDownloaded
  required property bool isDownloading
  required property bool isFavorite
  required property int downloadCount
  required property int resourceIndex
  property bool selected: false

  signal spawnRequested()
  signal downloadRequested(bool spawnAfter)
  signal favoriteToggled()

  readonly property bool isDark: Material.theme === Material.Dark
  readonly property bool isReady: !isFuel || isDownloaded
  readonly property bool hovered: cardMouse.containsMouse ||
    favButton.hovered || actionBtn.hovered || dlOnlyBtn.hovered

  function formatCount(n) {
    if (n >= 1000000)
      return (n / 1000000.0).toFixed(1) + "M"
    if (n >= 1000)
      return (n / 1000.0).toFixed(1) + "k"
    return n > 0 ? String(n) : ""
  }

  function statusBadgeText() {
    if (isDownloading)
      return "\u2B07 Downloading..."
    if (!isFuel)
      return "\u2713 Local"
    if (isDownloaded)
      return "\u2713 Cached"
    return formattedFileSize !== "" ? ("\u2601 " + formattedFileSize) : "\u2601 Fuel"
  }

  Rectangle {
    id: frame
    anchors.fill: parent
    anchors.margins: 5
    radius: 8
    clip: true
    color: card.isDark ?
      Material.color(Material.Grey, Material.Shade900) :
      Material.background
    border.width: (card.selected || card.hovered) ? 2 : 1
    border.color: (card.selected || card.hovered) ?
      Material.accent :
      (card.isDark ? Material.color(Material.Grey, Material.Shade700) :
                     Material.color(Material.Grey, Material.Shade300))

    ColumnLayout {
      anchors.fill: parent
      spacing: 0

      // Top thumbnail / fallback area
      Item {
        id: mediaArea
        Layout.fillWidth: true
        Layout.fillHeight: true

        Image {
          id: thumbImage
          anchors.fill: parent
          anchors.margins: 4
          asynchronous: true
          cache: true
          sourceSize.width: 240
          sourceSize.height: 160
          source: card.thumbnailUrl
          visible: card.thumbnailUrl !== "" && status === Image.Ready
          fillMode: Image.PreserveAspectFit
        }

        ResourceFallbackVisual {
          anchors.fill: parent
          anchors.margins: 4
          visible: !thumbImage.visible
          resourceName: card.resourceName
          category: card.category
          metaText: card.formattedFileSize !== "" ?
            card.formattedFileSize : (card.isFuel ? "FUEL" : "SDF")
          loading: card.thumbnailUrl !== "" &&
                   thumbImage.status === Image.Loading
        }

        // Top-left status pill
        Rectangle {
          anchors.top: parent.top
          anchors.left: parent.left
          anchors.margins: 8
          radius: 10
          height: 20
          width: statusLabel.implicitWidth + 14
          color: card.isDownloading ? Material.color(Material.Amber, Material.Shade800) :
                 (card.isReady ? Material.color(Material.Green, Material.Shade800) :
                                 "#CC263238")

          Label {
            id: statusLabel
            anchors.centerIn: parent
            text: card.statusBadgeText()
            color: "white"
            font.pointSize: 8
            font.bold: true
          }
        }

        // Top-right favorite star
        Rectangle {
          anchors.top: parent.top
          anchors.right: parent.right
          anchors.margins: 6
          width: 26
          height: 26
          radius: 13
          color: (card.isFavorite || card.hovered) ? "#99000000" : "transparent"

          ToolButton {
            id: favButton
            anchors.fill: parent
            padding: 0
            text: card.isFavorite ? "\u2605" : "\u2606"
            font.pointSize: 13
            palette.buttonText: card.isFavorite ? "#FFD54F" : "white"
            opacity: (card.isFavorite || card.hovered) ? 1.0 : 0.4
            ToolTip.text: card.isFavorite ? "Remove favorite" : "Add favorite"
            ToolTip.visible: hovered
            onClicked: card.favoriteToggled()
          }
        }

        // Hover quick-action overlay at bottom of thumbnail
        Rectangle {
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.bottom: parent.bottom
          anchors.margins: 6
          height: 30
          radius: 6
          color: "#D91E242B"
          visible: card.hovered && !card.isDownloading

          RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 6
            anchors.rightMargin: 6
            spacing: 4

            Button {
              id: actionBtn
              Layout.fillWidth: true
              Layout.preferredHeight: 24
              padding: 0
              flat: true
              text: card.isReady ? "+ Spawn" : "\u2B07 Download & Spawn"
              font.pointSize: 9
              font.bold: true
              palette.buttonText: "white"
              onClicked: {
                if (card.isReady)
                  card.spawnRequested()
                else
                  card.downloadRequested(true)
              }
            }

            ToolButton {
              id: dlOnlyBtn
              visible: !card.isReady
              implicitWidth: 24
              implicitHeight: 24
              padding: 0
              text: "\u2B07"
              font.pointSize: 11
              palette.buttonText: "#90CAF9"
              ToolTip.text: "Download only"
              ToolTip.visible: hovered
              onClicked: card.downloadRequested(false)
            }
          }
        }

        // Inline download progress bar
        ProgressBar {
          anchors.left: parent.left
          anchors.right: parent.right
          anchors.bottom: parent.bottom
          anchors.margins: 6
          indeterminate: true
          visible: card.isDownloading
        }
      }

      // Thin divider
      Rectangle {
        Layout.fillWidth: true
        height: 1
        color: card.isDark ? Material.color(Material.Grey, Material.Shade800) :
                             Material.color(Material.Grey, Material.Shade200)
      }

      // Bottom metadata footer
      ColumnLayout {
        Layout.fillWidth: true
        Layout.margins: 8
        Layout.topMargin: 6
        Layout.bottomMargin: 6
        spacing: 2

        Label {
          Layout.fillWidth: true
          text: card.resourceName
          font.pointSize: 10
          font.bold: true
          elide: Text.ElideRight
        }

        RowLayout {
          Layout.fillWidth: true
          spacing: 6

          Label {
            Layout.fillWidth: true
            text: card.isFuel ? card.ownerName : "Local model"
            font.pointSize: 9
            opacity: 0.65
            elide: Text.ElideMiddle
          }

          Label {
            visible: card.downloadCount > 0
            text: "\u2193 " + card.formatCount(card.downloadCount)
            font.pointSize: 8
            opacity: 0.65
          }
        }
      }
    }

    // Card click & drag-to-spawn MouseArea (underneath interactive buttons via z)
    MouseArea {
      id: cardMouse
      anchors.fill: parent
      z: -1
      hoverEnabled: true
      cursorShape: card.isDownloading ? Qt.BusyCursor : Qt.PointingHandCursor
      property point pressPos: Qt.point(0, 0)
      property bool dragSpawned: false

      onPressed: (mouse) => {
        pressPos = Qt.point(mouse.x, mouse.y)
        dragSpawned = false
      }

      onPositionChanged: (mouse) => {
        if (!pressed || dragSpawned || card.isDownloading)
          return
        var dx = mouse.x - pressPos.x
        var dy = mouse.y - pressPos.y
        if ((dx * dx + dy * dy) >
            (Qt.styleHints.startDragDistance * Qt.styleHints.startDragDistance)) {
          dragSpawned = true
          if (card.isReady)
            card.spawnRequested()
        }
      }

      onClicked: {
        if (dragSpawned || card.isDownloading)
          return
        if (card.isReady)
          card.spawnRequested()
        else
          card.downloadRequested(true)
      }
    }

    ToolTip {
      visible: cardMouse.containsMouse && !favButton.hovered && !dlOnlyBtn.hovered
      delay: 600
      text: card.resourceName +
            (card.isFuel ? (" (" + card.ownerName + ")") : " (Local)")
      enter: null
      exit: null
    }
  }
}
