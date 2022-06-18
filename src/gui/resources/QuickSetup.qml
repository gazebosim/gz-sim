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
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.3
import Qt.labs.folderlistmodel 2.1
import QtQuick.Window 2.2

  Rectangle {
      anchors.fill: parent
      width: 720
      height: 720
      id: quickSetup

      function loadWorld(fileURL){
        // Remove "file://" from the QML url.
        var url = fileURL.substring(7);
        QuickSetupHandler.SetStartingWorld(url)
      }

      function debug(f, ff, fff){
        console.log(f)
        console.log(ff)
        console.log(fff)
      }

      function loadFuelWorld(fileName){
        // Construct fuel URL
        var fuel_url = "https://fuel.ignitionrobotics.org/1.0/"
        fuel_url += fileName.split('.')[0] + "/worlds/" + fileName.split('.')[1]
        console.debug(fuel_url)
        QuickSetupHandler.SetStartingWorld(fuel_url)
        quickSetup.Window.window.close()
      }

      function getWorlds(){
        return "file://"+QuickSetupHandler.getWorldsPath()
      }

    RowLayout {
        id: layout
        // anchors.fill: parent
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalTop
        spacing: 6

          ColumnLayout {
            id: localColumn
            Layout.minimumHeight: 100
            Layout.fillWidth: true
            spacing: 0
              Rectangle {
                  color: 'transparent'
                  Layout.fillWidth: true
                  Layout.minimumWidth: 100
                  Layout.preferredWidth: 600
                  // Layout.maximumWidth: 700
                  Layout.minimumHeight: 150
                  Text {
                      anchors.centerIn: parent
                      text: "Gazebo logo here"
                  }
              }

            Rectangle {
                color: 'transparent'
                Layout.fillWidth: true
                  Layout.minimumWidth: 100
                  Layout.preferredWidth: 700
                  // Layout.maximumWidth: 700
                  Layout.minimumHeight: 400
                Text {
                    anchors.centerIn: parent
                    text: "Grid worlds thumbunails"
                }
                RowLayout{
                  spacing: 6
                  Rectangle {
                      visible: true
                      // color: "red";
                      width: 500;
                      height: 400;
                      FolderListModel {
                          id: folderModel
                          showDirs: false
                          nameFilters: ["*.png"]
                          folder: getWorlds()+ "/thumbnails/"
                      }

                      Component {
                          id: fileDelegate

                          World {
                              id: filePath
                              text: fileName
                              width: gridView.cellWidth - 5
                              height: gridView.cellHeight - 5
                              smooth: true
                              source: fileURL
                          }
                      }

                      GridView {
                          id: gridView
                          width: parent.width
                          height: parent.height - 200

                          anchors {
                              fill: parent
                              leftMargin: 5
                              topMargin: 5
                          }

                          cellWidth: width / 2
                          cellHeight: height / 2

                          model: folderModel
                          delegate: fileDelegate
                      }
                  }
                    ColumnLayout{
                      Rectangle {
                          color: "transparent";
                          width: 200; height: 200
                          
                          FolderListModel {
                              id: sdfsModel
                              showDirs: false
                              showFiles: true
                              folder: getWorlds()
                              nameFilters: [ "*.sdf" ]
                          }
                          
                          ComboBox {
                            id: comboBox
                              currentIndex : 2
                              model: sdfsModel
                              textRole: 'fileName'
                              width: parent.width
                              onCurrentIndexChanged: quickSetup.loadWorld(model.get(currentIndex, 'fileURL'))
                          }
                          MouseArea {
                            onClicked: comboBox.popup.close()
                          }
                        }
                      Rectangle { color: "transparent"; width: 200; height: 200 }
                    }
                }
            }
            Rectangle {
                color: 'transparent'
                Layout.fillWidth: true
                  Layout.minimumWidth: 100
                  Layout.preferredWidth: 700
                  // Layout.maximumWidth: 700
                  Layout.minimumHeight: 100
            }

              Rectangle {
                  color: 'transparent'
                  Layout.fillWidth: true
                    Layout.minimumWidth: 100
                    Layout.preferredWidth: 700
                    // Layout.maximumWidth: 700
                    Layout.minimumHeight: 50
                  RowLayout {
                      id: skip
                      // anchors.fill: parent
                      anchors.horizontalCenter: parent.horizontalCenter
                      anchors.verticalCenter: parent.verticalTop
                      spacing: 6
                      Button {
                        id: closeButton
                        visible: true
                        text: "Skip"
                        // anchors.centerIn: parent
                        Layout.minimumWidth: 100
                        Layout.leftMargin: 10

                        onClicked: {
                          quickSetup.loadWorld("")
                          quickSetup.Window.window.close()
                        }

                        Material.background: Material.primary
                        ToolTip.visible: hovered
                        ToolTip.delay: tooltipDelay
                        ToolTip.timeout: tooltipTimeout
                        ToolTip.text: qsTr("Skip")
                      }
                      Button {
                        id: next
                        visible: true
                        text: "Next"
                        // anchors.centerIn: parent
                        Layout.minimumWidth: 100
                        Layout.leftMargin: 10

                        onClicked: {
                          quickSetup.Window.window.close()
                        }

                        Material.background: Material.primary
                        ToolTip.visible: hovered
                        ToolTip.delay: tooltipDelay
                        ToolTip.timeout: tooltipTimeout
                        ToolTip.text: qsTr("Next")
                      }
                      CheckBox {
                        text: "Don't show again"
                        Layout.fillWidth: true
                        onClicked: {
                          console.debug("not yet implmented")
                        }
                      }
                }
              }
          }
        }
  }
