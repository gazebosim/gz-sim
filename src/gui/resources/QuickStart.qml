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
import QtQuick.Layouts 1.3
import QtQml.Models 2.3
import Qt.labs.folderlistmodel 2.1
import QtQuick.Window 2.2
import "qrc:/qml"

Rectangle {

  /**
   * Color for search bar
   */
  property color searchColor: (Material.theme == Material.Light) ?
      Material.color(Material.Grey, Material.Shade200):
      Material.color(Material.Grey, Material.Shade900);


  id: quickStart
  anchors.fill: parent
  property var selectedWorld: ""

  function changeDefault(checked) {
    QuickStartHandler.SetShowAgain(checked);
  }

  function loadWorld(file){
    if (file === quickStart.selectedWorld) {
      openWorld.enabled = false;
      quickStart.selectedWorld = "";
    } else {
      QuickStartHandler.SetStartingWorld(file);
      openWorld.enabled = true;
      quickStart.selectedWorld = file;
    }
  }

  function loadFuelWorld(fileName, owner){
    if (quickStart.selectedWorld === fileName) {
      openWorld.enabled = false;
      quickStart.selectedWorld = '';
    } else {

      quickStart.selectedWorld = fileName;

      if (fileName === "empty") {
        QuickStartHandler.SetStartingWorld("empty.sdf");
        openWorld.enabled = true;
      } else {
        // Construct fuel URL
        var fuelUrl = "https://fuel.gazebosim.org/1.0/" + owner + "/worlds/";
        fuelUrl += fileName;
        QuickStartHandler.SetStartingWorld(fuelUrl);
        openWorld.enabled = true;
      }
    }
  }

  function getWorlds(){
    return "file://" + QuickStartHandler.WorldsPath()
  }

  function getColor(fileName){
    if(fileName == selectedWorld)
      return Material.primary;
    return "#e0e0e0";
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
          Text{
            text: QuickStartHandler.Distribution()
          }
          Text{
            text: 'v ' + QuickStartHandler.SimVersion()
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
      height: 360
      RowLayout {
        spacing: 0
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

          FolderListModel {
            id: folderModel
            showDirs: false
            nameFilters: ["*.png"]
            folder: getWorlds() + "/thumbnails/"
          }

          Component {
            id: fileDelegate

            FuelThumbnail {
              id: filePath
              text: fileName.split('.')[1]
              owner: fileName.split('.')[0]
              width: 220
              height: 150
              smooth: true
              source: fileURL
              border.color: getColor(fileName.split('.')[1])
            }
          }
          GridView {
            id: gridView
            width: parent.width
            height: parent.height
            interactive: false

            anchors {
              fill: parent
              leftMargin: 5
              topMargin: 5
            }

            cellWidth: 240
            cellHeight: 205

            model: folderModel
            delegate: fileDelegate
          }
        }

        // SDF file list with search bar
        Rectangle {
          color: 'transparent'
          width: 200
          Layout.fillHeight: true
          Layout.topMargin: 5
          border {
            width: 1
            color: searchColor
          }

          ColumnLayout {
            anchors.fill: parent
            spacing: 0

            Rectangle {
              id: searchSortBar
              color: searchColor
              height: 50
              width: parent.width
              RowLayout {
                id: rowLayout
                anchors.fill: parent
                spacing: 0
                Rectangle {
                  color: "transparent"
                  height: 25
                  width: 25
                  Layout.leftMargin: 5
                  Image {
                    id: searchIcon
                    source: "images/search.svg"
                    anchors.verticalCenter: parent.verticalCenter
                  }
                }
                TextField {
                  id: searchField
                  Layout.fillHeight: true
                  Layout.preferredWidth: parent.width - 50
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
                width: parent.width-11
                x: 1
                text: fileName
                highlighted: selectedWorld == fileName
                onClicked: {
                  quickStart.loadWorld(fileName);
                }
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

              Layout.fillHeight: true
              width: parent.width
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
      Layout.fillHeight: true
      height: 60
      color: 'transparent'

      RowLayout {
        anchors {
          fill: parent
          leftMargin: 20
          rightMargin: 20
        }
        CheckBox {
          id: showByDefault
          text: "Don't show this dialog again"
          Layout.fillWidth: true
          onClicked: {
            quickStart.changeDefault(showByDefault.checked)
          }
        }
        Button {
          id: openWorld
          visible: true
          text: "Run"
          enabled: false
          onClicked: {
            quickStart.Window.window.close()
          }
          background: Rectangle {
            implicitWidth: 60
            implicitHeight: 40
            radius: 4
            color: openWorld.hovered ? "#efefef" : 'transparent'
            border {
              width: 2
              color: openWorld.enabled ? Material.primary : "#efefef"
            }
         }
        }
      }
    }
  }
}
