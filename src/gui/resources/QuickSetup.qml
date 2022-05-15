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
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.3
import Qt.labs.folderlistmodel 2.1

/**
 * Quick Setup
 */
// Item{
//   // id: quickSetup
//   x: Math.round((parent.width - width) / 2)
//   y: Math.round((parent.height - height) / 2)

  Rectangle {
      width: 420
      height: 400
      id: quickSetup

      function loadWorld(fileName, fileURL_, fileIsDir){
        console.log(fileURL_)
        console.log(fileName)
        console.log(fileIsDir)
        TmpIface.OnLoadWorldFromDir(fileURL)
      }

      function getThumbnail(fileURL, fileIsDir){
        if (fileIsDir)
          return fileURL + "/thumbnails/0.png";
        else
          return "";
      }

      // color: Material.color(Material.Grey, Material.Shade600);
      Component.onCompleted: function(){console.log("component completed")}
    
      Column {
        anchors.fill: parent
        anchors.margins: 10

        Row {
          Column{
            Label {
            text: "Worlds"
            }
          Rectangle {
          width: 100
          height: 300
          color: "transparent"


              ListView {
                anchors.fill: parent

                FolderListModel {
                    id: folderModel
                    showDirs: true
                    showFiles: true
                    showDirsFirst: true
                    folder: "file:///home/m/repos/citadel/src/ign-gazebo/examples/worlds/"
                    nameFilters: [ "*.sdf" ]
                }

                model: folderModel
                delegate: World {
                  width: parent.width
                  height: 100
                  id: filePath
                  // text: filePath
                  text: fileName
                  source: quickSetup.getThumbnail(fileURL, fileIsDir)
                  onClicked: quickSetup.loadWorld(fileName, fileURL, fileIsDir)
                }

                ScrollIndicator.vertical: ScrollIndicator {
                  active: true;
                  onActiveChanged: {
                    active = true;
                  }
                }
              }
            }
          }
          Column{
            CheckBox {
              text: "Don't show again"
              Layout.fillWidth: true
              // checked: sdfGenConfig.saveFuelModelVersion
              onClicked: {
                sdfGenConfig.saveFuelModelVersion = checked
            }
          }
          Button {
            id: closeButton
            visible: true
            text: "Skip"
            Layout.alignment : Qt.AlignVCenter
            Layout.minimumWidth: width
            Layout.leftMargin: 10

            onClicked: {
              quickSetup.close();
            }

            Material.background: Material.primary
            ToolTip.visible: hovered
            ToolTip.delay: tooltipDelay
            ToolTip.timeout: tooltipTimeout
            ToolTip.text: qsTr("Skip")
          }
        }
      }

    }
  }
// }