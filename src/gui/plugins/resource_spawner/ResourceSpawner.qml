/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
import QtQuick.Window 2.2
import QtQuick.Controls
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.3

Rectangle {
  id: resourceSpawner
  color: Material.background
  Layout.minimumWidth: 440
  Layout.minimumHeight: 380
  anchors.fill: parent

  readonly property bool isDark: Material.theme === Material.Dark
  readonly property color surfaceColor: isDark ?
    Material.color(Material.Grey, Material.Shade900) :
    Material.color(Material.Grey, Material.Shade100)
  readonly property color subtleBarColor: isDark ?
    Material.color(Material.Grey, Material.Shade800) :
    Material.color(Material.Grey, Material.Shade200)
  readonly property color borderColor: isDark ?
    Material.color(Material.Grey, Material.Shade700) :
    Material.color(Material.Grey, Material.Shade300)

  // Active unified source pill: "all", "local", "fuel", "cached", "favorites"
  property string activeSource: "fuel"
  property string currentOwner: "openrobotics"
  property string currentLocalPath: ""
  property bool listMode: false
  property bool showRecentStrip: true

  function selectSource(src) {
    activeSource = src
    _ResourceSpawner.OnSourceFilterChosen(src)
    if (src === "fuel")
      _ResourceSpawner.OnOwnerClicked(currentOwner)
    else if (src === "local" && currentLocalPath !== "")
      _ResourceSpawner.OnPathClicked(currentLocalPath)
    _ResourceSpawner.DisplayResources()
    gridView.currentIndex = -1
    listView.currentIndex = -1
  }

  ColumnLayout {
    anchors.fill: parent
    spacing: 0

    // Row 1: Search, Sort, View-mode toggle
    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 52
      color: surfaceColor

      RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        spacing: 8

        // Rounded search input
        Rectangle {
          Layout.fillWidth: true
          Layout.preferredHeight: 36
          radius: 18
          color: Material.background
          border.width: searchField.activeFocus ? 2 : 1
          border.color: searchField.activeFocus ? Material.accent : borderColor

          RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 12
            anchors.rightMargin: 8
            spacing: 6

            Image {
              source: "qrc:/qml/images/search.svg"
              sourceSize.width: 16
              sourceSize.height: 16
              opacity: 0.6
            }

            TextInput {
              id: searchField
              Layout.fillWidth: true
              verticalAlignment: TextInput.AlignVCenter
              selectByMouse: true
              clip: true
              color: isDark ? "white" : "black"
              font.pointSize: 10

              Text {
                anchors.fill: parent
                verticalAlignment: Text.AlignVCenter
                text: "Search models, owners, categories..."
                color: isDark ? "#88ffffff" : "#66000000"
                font.pointSize: 10
                visible: searchField.text === ""
              }

              onTextEdited: {
                _ResourceSpawner.OnSearchEntered(searchField.text)
                _ResourceSpawner.DisplayResources()
              }
            }

            ToolButton {
              visible: searchField.text !== ""
              implicitWidth: 24
              implicitHeight: 24
              padding: 0
              text: "\u2715"
              font.pointSize: 9
              onClicked: {
                searchField.text = ""
                _ResourceSpawner.OnSearchEntered("")
                _ResourceSpawner.DisplayResources()
              }
            }
          }
        }

        // Sort ComboBox
        ComboBox {
          id: sortComboBox
          Layout.preferredWidth: 145
          Layout.preferredHeight: 36
          model: ["Most Recent", "Popular", "A - Z", "Z - A", "Downloaded"]
          onActivated: {
            _ResourceSpawner.OnSortChosen(currentText)
            _ResourceSpawner.DisplayResources()
          }
        }

        // Grid / List toggle
        Rectangle {
          Layout.preferredHeight: 34
          Layout.preferredWidth: 68
          radius: 6
          color: subtleBarColor
          border.color: borderColor
          border.width: 1

          RowLayout {
            anchors.fill: parent
            anchors.margins: 2
            spacing: 2

            ToolButton {
              Layout.fillWidth: true
              Layout.fillHeight: true
              padding: 0
              text: "\u25A6"
              font.pointSize: 13
              ToolTip.text: "Grid view"
              ToolTip.visible: hovered
              background: Rectangle {
                radius: 4
                color: !resourceSpawner.listMode ? Material.accent : "transparent"
              }
              onClicked: resourceSpawner.listMode = false
            }

            ToolButton {
              Layout.fillWidth: true
              Layout.fillHeight: true
              padding: 0
              text: "\u2630"
              font.pointSize: 12
              ToolTip.text: "List view"
              ToolTip.visible: hovered
              background: Rectangle {
                radius: 4
                color: resourceSpawner.listMode ? Material.accent : "transparent"
              }
              onClicked: resourceSpawner.listMode = true
            }
          }
        }
      }
    }

    // Row 2: Unified source filter pills
    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 42
      color: surfaceColor

      Flickable {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        contentWidth: pillRow.implicitWidth
        contentHeight: height
        clip: true
        flickableDirection: Flickable.HorizontalFlick

        RowLayout {
          id: pillRow
          anchors.verticalCenter: parent.verticalCenter
          spacing: 6

          // All pill
          Button {
            text: "All"
            flat: activeSource !== "all"
            highlighted: activeSource === "all"
            implicitHeight: 30
            font.pointSize: 9
            onClicked: selectSource("all")
          }

          // Local pill (opens path selector if clicked while active and multiple paths exist)
          Button {
            id: localPill
            text: "Local (" + _ResourceSpawner.localCount + ")" +
                  (PathList.rowCount() > 1 ? " \u25BE" : "")
            flat: activeSource !== "local"
            highlighted: activeSource === "local"
            implicitHeight: 30
            font.pointSize: 9
            onClicked: {
              if (activeSource === "local" && PathList.rowCount() > 1)
                localPathPopup.open()
              else {
                currentLocalPath = ""
                selectSource("local")
              }
            }
          }

          // Fuel owner pill (switches to Fuel, or opens Owner picker popup)
          Button {
            id: fuelPill
            text: "Fuel: " + currentOwner + " \u25BE"
            flat: activeSource !== "fuel"
            highlighted: activeSource === "fuel"
            implicitHeight: 30
            font.pointSize: 9
            onClicked: {
              if (activeSource === "fuel")
                ownerPopup.open()
              else
                selectSource("fuel")
            }
          }

          // Cached pill
          Button {
            text: "Cached"
            flat: activeSource !== "cached"
            highlighted: activeSource === "cached"
            implicitHeight: 30
            font.pointSize: 9
            onClicked: selectSource("cached")
          }

          // Favorites pill
          Button {
            text: "\u2605 Favorites"
            flat: activeSource !== "favorites"
            highlighted: activeSource === "favorites"
            implicitHeight: 30
            font.pointSize: 9
            onClicked: selectSource("favorites")
          }
        }
      }

      Rectangle {
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        height: 1
        color: borderColor
      }
    }

    // Row 3: Collapsible Recently Spawned quick-access strip
    Rectangle {
      id: recentStrip
      visible: RecentResourceList.totalCount > 0
      Layout.fillWidth: true
      Layout.preferredHeight: showRecentStrip ? 56 : 24
      color: subtleBarColor

      ColumnLayout {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        anchors.topMargin: 2
        anchors.bottomMargin: 4
        spacing: 2

        RowLayout {
          Layout.fillWidth: true
          Label {
            text: "Recent (" + RecentResourceList.totalCount + ")"
            font.pointSize: 9
            font.bold: true
            opacity: 0.75
          }
          Item { Layout.fillWidth: true }
          ToolButton {
            text: showRecentStrip ? "Hide" : "Show"
            font.pointSize: 8
            implicitHeight: 20
            padding: 2
            onClicked: showRecentStrip = !showRecentStrip
          }
        }

        ListView {
          visible: showRecentStrip
          Layout.fillWidth: true
          Layout.fillHeight: true
          orientation: ListView.Horizontal
          spacing: 6
          clip: true
          model: RecentResourceList

          delegate: Rectangle {
            height: 26
            width: Math.min(160, recentLabel.implicitWidth + 22)
            radius: 13
            color: recentMa.containsMouse ? Material.accent : surfaceColor
            border.color: borderColor
            border.width: 1

            Label {
              id: recentLabel
              anchors.centerIn: parent
              width: parent.width - 14
              text: model.name
              elide: Text.ElideRight
              font.pointSize: 9
              horizontalAlignment: Text.AlignHCenter
            }

            MouseArea {
              id: recentMa
              anchors.fill: parent
              hoverEnabled: true
              cursorShape: Qt.PointingHandCursor
              onClicked: {
                _ResourceSpawner.RecordRecentSpawn(
                  model.sdf, model.name, model.owner, model.thumbnailUrl)
                _ResourceSpawner.OnResourceSpawn(model.sdf)
              }
            }
          }
        }
      }
    }

    // Main content area: Responsive GridView or compact ListView
    Item {
      Layout.fillWidth: true
      Layout.fillHeight: true

      // Empty / loading state
      ColumnLayout {
        anchors.centerIn: parent
        visible: ResourceList.totalCount === 0
        spacing: 8

        BusyIndicator {
          Layout.alignment: Qt.AlignHCenter
          running: _ResourceSpawner.isFetching
          visible: _ResourceSpawner.isFetching
        }

        Label {
          Layout.alignment: Qt.AlignHCenter
          text: _ResourceSpawner.isFetching ?
            "Loading models from Fuel..." :
            (searchField.text !== "" ?
              "No models match \"" + searchField.text + "\"" :
              (activeSource === "favorites" ?
                "No favorites yet — click \u2606 on any card to pin it here." :
                "No models found in this view."))
          opacity: 0.7
          font.pointSize: 11
        }
      }

      // Responsive GridView
      GridView {
        id: gridView
        anchors.fill: parent
        anchors.margins: 6
        visible: !resourceSpawner.listMode
        model: ResourceList
        readonly property int columns: Math.max(1, Math.floor(width / 195))
        cellWidth: Math.floor(width / columns)
        cellHeight: 185
        currentIndex: -1
        clip: true

        ScrollBar.vertical: ScrollBar { active: true }

        delegate: ResourceCard {
          required property int index
          required property var model

          width: gridView.cellWidth
          height: gridView.cellHeight
          resourceName: model.name
          ownerName: model.owner
          sdfPath: model.sdf
          thumbnailUrl: model.thumbnailUrl
          category: model.category
          formattedFileSize: model.formattedFileSize
          isFuel: model.isFuel
          isDownloaded: model.isDownloaded
          isDownloading: model.isDownloading
          isFavorite: model.isFavorite
          downloadCount: model.downloadCount
          resourceIndex: model.index
          selected: gridView.currentIndex === index

          onSpawnRequested: {
            _ResourceSpawner.RecordRecentSpawn(
              model.sdf, model.name, model.owner, model.thumbnailUrl)
            _ResourceSpawner.OnResourceSpawn(model.sdf)
            gridView.currentIndex = index
          }

          onDownloadRequested: (spawnAfter) => {
            _ResourceSpawner.OnDownloadFuelResource(
              model.sdf, model.name, model.owner, model.index, spawnAfter)
          }

          onFavoriteToggled: {
            _ResourceSpawner.ToggleFavorite(model.owner, model.name)
          }
        }
      }

      // Compact ListView mode
      ListView {
        id: listView
        anchors.fill: parent
        anchors.margins: 6
        visible: resourceSpawner.listMode
        model: ResourceList
        spacing: 4
        currentIndex: -1
        clip: true

        ScrollBar.vertical: ScrollBar { active: true }

        delegate: Rectangle {
          width: listView.width - 10
          height: 54
          radius: 6
          color: listView.currentIndex === index ? Material.accent :
                 (rowMa.containsMouse ? subtleBarColor : surfaceColor)
          border.color: borderColor
          border.width: 1

          RowLayout {
            anchors.fill: parent
            anchors.margins: 6
            spacing: 10

            Item {
              Layout.preferredWidth: 52
              Layout.preferredHeight: 40

              Image {
                id: rowThumb
                anchors.fill: parent
                asynchronous: true
                cache: true
                sourceSize.width: 64
                sourceSize.height: 48
                source: model.thumbnailUrl
                visible: model.thumbnailUrl !== "" && status === Image.Ready
                fillMode: Image.PreserveAspectFit
              }

              ResourceFallbackVisual {
                anchors.fill: parent
                visible: !rowThumb.visible
                resourceName: model.name
                category: model.category
                loading: model.thumbnailUrl !== "" &&
                         rowThumb.status === Image.Loading
              }
            }

            ColumnLayout {
              Layout.fillWidth: true
              spacing: 1
              Label {
                Layout.fillWidth: true
                text: model.name
                font.bold: true
                font.pointSize: 10
                elide: Text.ElideRight
              }
              Label {
                Layout.fillWidth: true
                text: (model.isFuel ? model.owner : "Local") +
                      (model.formattedFileSize !== "" ?
                        (" \u00B7 " + model.formattedFileSize) : "")
                font.pointSize: 8
                opacity: 0.7
                elide: Text.ElideRight
              }
            }

            BusyIndicator {
              visible: model.isDownloading
              running: model.isDownloading
              implicitWidth: 24
              implicitHeight: 24
            }

            Button {
              visible: !model.isDownloading
              text: (!model.isFuel || model.isDownloaded) ? "+ Spawn" : "\u2B07 Get"
              implicitHeight: 28
              font.pointSize: 9
              onClicked: {
                if (!model.isFuel || model.isDownloaded) {
                  _ResourceSpawner.RecordRecentSpawn(
                    model.sdf, model.name, model.owner, model.thumbnailUrl)
                  _ResourceSpawner.OnResourceSpawn(model.sdf)
                  listView.currentIndex = index
                } else {
                  _ResourceSpawner.OnDownloadFuelResource(
                    model.sdf, model.name, model.owner, model.index, true)
                }
              }
            }

            ToolButton {
              implicitWidth: 28
              implicitHeight: 28
              padding: 0
              text: model.isFavorite ? "\u2605" : "\u2606"
              font.pointSize: 13
              onClicked: _ResourceSpawner.ToggleFavorite(model.owner, model.name)
            }
          }

          MouseArea {
            id: rowMa
            anchors.fill: parent
            z: -1
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onClicked: {
              if (model.isDownloading)
                return
              if (!model.isFuel || model.isDownloaded) {
                _ResourceSpawner.RecordRecentSpawn(
                  model.sdf, model.name, model.owner, model.thumbnailUrl)
                _ResourceSpawner.OnResourceSpawn(model.sdf)
                listView.currentIndex = index
              } else {
                _ResourceSpawner.OnDownloadFuelResource(
                  model.sdf, model.name, model.owner, model.index, true)
              }
            }
          }
        }
      }
    }

    // Footer status bar
    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 28
      color: surfaceColor

      Rectangle {
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 1
        color: borderColor
      }

      RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 10
        anchors.rightMargin: 10
        spacing: 8

        Label {
          text: "Showing " + ResourceList.totalCount + " model" +
                (ResourceList.totalCount === 1 ? "" : "s") +
                " \u00B7 Click or drag onto the 3D scene to place"
          font.pointSize: 9
          opacity: 0.7
          elide: Text.ElideRight
          Layout.fillWidth: true
        }

        BusyIndicator {
          visible: _ResourceSpawner.isFetching
          running: _ResourceSpawner.isFetching
          implicitWidth: 18
          implicitHeight: 18
        }
      }
    }
  }

  // Fuel Owner selector & manager popup
  Popup {
    id: ownerPopup
    x: fuelPill.x + 10
    y: 94
    width: 280
    height: Math.min(320, ownerListPopup.contentHeight + 90)
    padding: 8
    modal: false
    focus: true
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

    ColumnLayout {
      anchors.fill: parent
      spacing: 6

      Label {
        text: "Fuel Owners"
        font.bold: true
        font.pointSize: 10
      }

      ListView {
        id: ownerListPopup
        Layout.fillWidth: true
        Layout.fillHeight: true
        model: OwnerList
        clip: true
        spacing: 2

        delegate: Rectangle {
          width: ownerListPopup.width
          height: 32
          radius: 4
          color: currentOwner === model.path ? Material.accent :
                 (ownerRowMa.containsMouse ? subtleBarColor : "transparent")

          RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 8
            anchors.rightMargin: 4
            spacing: 4

            Label {
              Layout.fillWidth: true
              text: model.path
              elide: Text.ElideMiddle
              font.pointSize: 10
            }

            ToolButton {
              visible: !_ResourceSpawner.IsDefaultOwner(model.path)
              text: "\u2715"
              implicitWidth: 24
              implicitHeight: 24
              padding: 0
              onClicked: _ResourceSpawner.RemoveOwner(model.path)
            }
          }

          MouseArea {
            id: ownerRowMa
            anchors.fill: parent
            z: -1
            hoverEnabled: true
            cursorShape: Qt.PointingHandCursor
            onClicked: {
              currentOwner = model.path
              selectSource("fuel")
              ownerPopup.close()
            }
          }
        }
      }

      RowLayout {
        Layout.fillWidth: true
        spacing: 6

        TextField {
          id: ownerInput
          Layout.fillWidth: true
          placeholderText: "Add Fuel owner..."
          selectByMouse: true
          font.pointSize: 10

          function processInput() {
            if (text.trim() !== "" && _ResourceSpawner.AddOwner(text.trim())) {
              currentOwner = text.trim()
              selectSource("fuel")
              text = ""
              ownerPopup.close()
            }
          }
          onAccepted: processInput()
        }

        Button {
          text: "+"
          implicitWidth: 36
          onClicked: ownerInput.processInput()
        }
      }
    }
  }

  // Local path selector popup
  Popup {
    id: localPathPopup
    x: localPill.x + 10
    y: 94
    width: 320
    height: Math.min(260, localPathList.contentHeight + 60)
    padding: 8
    focus: true
    closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

    ColumnLayout {
      anchors.fill: parent
      spacing: 4

      Button {
        Layout.fillWidth: true
        flat: true
        text: "All local paths (" + _ResourceSpawner.localCount + ")"
        onClicked: {
          currentLocalPath = ""
          selectSource("local")
          localPathPopup.close()
        }
      }

      ListView {
        id: localPathList
        Layout.fillWidth: true
        Layout.fillHeight: true
        model: PathList
        clip: true
        spacing: 2

        delegate: Button {
          width: localPathList.width
          flat: currentLocalPath !== model.path
          highlighted: currentLocalPath === model.path
          text: model.path
          onClicked: {
            currentLocalPath = model.path
            selectSource("local")
            localPathPopup.close()
          }
        }
      }
    }
  }

  // Dialog for error messages
  Dialog {
    id: messageDialog
    width: 360
    height: 150
    parent: resourceSpawner.Window.window ?
      resourceSpawner.Window.window.contentItem : resourceSpawner
    x: Math.round((parent.width - width) / 2)
    y: Math.round((parent.height - height) / 2)
    modal: true
    focus: true
    title: "Error"
    standardButtons: Dialog.Ok
    contentItem: Text {
      text: ""
      wrapMode: Text.WordWrap
    }
  }

  Connections {
    target: _ResourceSpawner
    function onResourceSpawnerError(_errorMsg) {
      messageDialog.contentItem.text = _errorMsg
      messageDialog.visible = true
    }
  }
}
