/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs
import QtQuick.Layouts 1.3
import QtQml.Models 2.3
import Qt.labs.folderlistmodel 2.1
import QtQuick.Window 2.2
import "qrc:/qml"

Rectangle {

  property bool isDark: Material.theme === Material.Dark

  /**
   * Color for search bar
   */
  property color searchColor: isDark ?
      Material.color(Material.Grey, Material.Shade900) :
      Material.color(Material.Grey, Material.Shade200);

  property color borderColor: isDark ?
      Material.color(Material.Grey, Material.Shade700) : "#e0e0e0";

  property color textColor: isDark ? "#f1f5f9" : "#263238";

  id: quickStart
  objectName: "quickStart"
  anchors.fill: parent
  color: Material.background
  focus: true
  property var selectedWorld: ""

  Shortcut {
    sequence: StandardKey.Cancel
    onActivated: {
      QuickStartHandler.SetStartingWorld("");
      Qt.callLater(function() {
        if (quickStart.Window.window) {
          quickStart.Window.window.close();
        }
      });
    }
  }

  Shortcut {
    sequences: ["Return", "Enter"]
    enabled: openWorld.enabled && !searchField.activeFocus
    onActivated: quickStart.launchSelected()
  }

  FileDialog {
    id: openWorldFileDialog
    title: qsTr("Open World File")
    fileMode: FileDialog.OpenFile
    nameFilters: [
      qsTr("SDF / World files (*.sdf *.world)"),
      qsTr("All files (*)")
    ]
    onAccepted: {
      var localPath = QuickStartHandler.LocalPathFromUrl(selectedFile);
      quickStart.launchWorld(localPath, localPath);
    }
  }

  function changeDefault(checked) {
    QuickStartHandler.SetShowAgain(checked);
  }

  // Select the world identified by key to launch uri, or deselect it if it
  // is already selected or uri is empty. A deselected world is not launched
  // on close.
  function selectWorld(key, uri) {
    var select = key !== quickStart.selectedWorld && uri !== "";
    quickStart.selectedWorld = select ? key : "";
    openWorld.enabled = select;
    QuickStartHandler.SetStartingWorld(select ? uri : "");
  }

  // Select the world identified by key and launch uri right away, unless uri
  // is empty.
  function launchWorld(key, uri) {
    if (uri === "")
      return;
    quickStart.selectedWorld = key;
    QuickStartHandler.SetStartingWorld(uri);
    openWorld.enabled = true;
    quickStart.launchSelected();
  }

  function launchSelected() {
    if (!openWorld.enabled) {
      return;
    }
    Qt.callLater(function() {
      if (quickStart.Window.window) {
        quickStart.Window.window.close();
      }
    });
  }

  function loadWorld(file){
    selectWorld(file, file);
  }

  function loadCatalogWorld(world) {
    var uri = QuickStartHandler.ResolveWorldUri(world);
    if (uri === "")
      console.error("QuickStart: cannot resolve world [" + world.id + "]");
    selectWorld(world.id, uri);
  }

  function selectAndLaunchCatalogWorld(world) {
    launchWorld(world.id, QuickStartHandler.ResolveWorldUri(world));
  }

  function getWorlds(){
    return "file://" + QuickStartHandler.WorldsPath()
  }

  function getColor(worldId){
    if(worldId == selectedWorld)
      return Material.primary;
    return borderColor;
  }

  ColumnLayout {
    anchors.fill: parent
    spacing: 0

    // Top row, which holds the logo and version.
    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 100
      color:'transparent'

      RowLayout {
        anchors.fill: parent
        Rectangle {
          color: 'transparent'
          Layout.fillWidth: true
          Layout.preferredWidth: 960
          Layout.preferredHeight: 100
          Layout.minimumHeight: 100
          Image{
            source: "images/gazebo_horz_pos_topbar.svg"
            fillMode: Image.PreserveAspectFit
            x: 30
            y: (parent.height - height) / 2
          }
        }
        ColumnLayout {
          Text {
            text: QuickStartHandler.Distribution()
            color: quickStart.textColor
            font.weight: Font.DemiBold
            renderType: Text.NativeRendering
          }
          Text {
            text: 'v ' + QuickStartHandler.SimVersion()
            color: quickStart.textColor
            renderType: Text.NativeRendering
          }
        }
        Item {
          width: 30
        }
      }
    }

    // Middle row, which holds the world selection elements.
    Rectangle {
      Layout.fillWidth: true
      Layout.fillHeight: true
      color: "transparent"
      RowLayout {
        spacing: 12
        anchors {
          fill: parent
          leftMargin: 15
          rightMargin: 20
        }

        // Card grid view
        Rectangle {
          color: 'transparent'
          Layout.fillWidth: true
          Layout.fillHeight: true

          Component {
            id: fileDelegate

            FuelThumbnail {
              text: modelData.title
              worldData: modelData
              width: 220
              height: 156
              source: QuickStartHandler.ResolveThumbnailUrl(modelData)
              border.color: getColor(modelData.id)
            }
          }
          GridView {
            id: gridView
            objectName: "featuredGrid"
            clip: true

            anchors {
              fill: parent
              leftMargin: 5
              topMargin: 5
            }

            cellWidth: 240
            cellHeight: 176

            model: QuickStartHandler.FeaturedWorlds()
            delegate: fileDelegate

            ScrollBar.vertical: ScrollBar {
              policy: ScrollBar.AsNeeded
            }
          }
        }

        // SDF file list with search bar
        Rectangle {
          color: 'transparent'
          Layout.preferredWidth: 240
          Layout.fillHeight: true
          Layout.topMargin: 5
          border {
            width: 1
            color: borderColor
          }

          ColumnLayout {
            anchors.fill: parent
            spacing: 0

            Rectangle {
              id: searchSortBar
              color: searchColor
              Layout.fillWidth: true
              Layout.preferredHeight: 48
              RowLayout {
                id: rowLayout
                anchors.fill: parent
                anchors.leftMargin: 8
                anchors.rightMargin: 8
                spacing: 6
                Image {
                  id: searchIcon
                  source: "images/search.svg"
                  Layout.preferredWidth: 18
                  Layout.preferredHeight: 18
                  fillMode: Image.PreserveAspectFit
                }
                TextField {
                  id: searchField
                  Layout.fillWidth: true
                  placeholderText: qsTr("Search local .sdf...")
                  selectByMouse: true
                  onTextEdited: {
                    sdfFileModel.update();
                  }
                }
              }
            }

            Component {
              id: sdfFileDelegate

              ItemDelegate {
                width: pluginMenuListView.width
                text: fileName
                highlighted: selectedWorld == fileName
                onClicked: {
                  quickStart.loadWorld(fileName);
                }
                onDoubleClicked: quickStart.launchWorld(fileName, fileName)
              }
            }

            GzSortFilterModel {
              id: sdfFileModel

              lessThan: function(left, right) {
                var leftStr = left.fileName.toLowerCase();
                var rightStr = right.fileName.toLowerCase();
                return leftStr < rightStr;
              }

              filterAcceptsItem: function(item) {
                var itemStr = item.fileName.toLowerCase();
                var filterStr = searchField.text.toLowerCase();
                return itemStr.includes(filterStr);
              }

              model: FolderListModel {
                showDirs: false
                showFiles: true
                folder: getWorlds()
                nameFilters: [ "*.sdf" ]
              }

              delegate: sdfFileDelegate
            }

            ListView {
              id: pluginMenuListView

              Layout.fillWidth: true
              Layout.fillHeight: true
              clip: true
              model: sdfFileModel

              ScrollBar.vertical: ScrollBar {
                active: true
              }
            }
          }
        }
      }
    }

    // Bottom row, which holds the run button and don't show checkbox.
    Rectangle {
      Layout.fillWidth: true
      Layout.preferredHeight: 64
      color: 'transparent'

      RowLayout {
        anchors {
          fill: parent
          leftMargin: 20
          rightMargin: 20
        }
        spacing: 10
        CheckBox {
          id: showByDefault
          text: qsTr("Don't show this dialog again")
          Layout.fillWidth: true
          onClicked: {
            quickStart.changeDefault(showByDefault.checked)
          }
        }
        Button {
          id: openFileBtn
          text: qsTr("Open File...")
          flat: true
          onClicked: openWorldFileDialog.open()
        }
        Button {
          id: emptyWorldBtn
          text: qsTr("Empty World")
          flat: true
          onClicked: quickStart.launchWorld("empty.sdf", "empty.sdf")
        }
        Button {
          id: openWorld
          visible: true
          text: qsTr("Run")
          enabled: false
          highlighted: true
          onClicked: quickStart.launchSelected()
        }
      }
    }
  }
}
