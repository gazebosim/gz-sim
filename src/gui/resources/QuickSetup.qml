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

ApplicationWindow
{
  title: qsTr("Quick Setup")
  width: 1200
  height: 1000
  minimumWidth: 300
  minimumHeight: 300
  visible: true
  id: window
  objectName: "window"
  font.family: "Roboto"

  Rectangle {
      // width: 420
      // height: 400
      id: quickSetup

      function loadWorld(fileName, fileURL_, fileIsDir){
        console.log(fileURL_)
        console.log(fileName)
        console.log(fileIsDir)
        QuickSetupHandler.SetStartingWorld(fileURL_)
        window.close()
      }

      function getWorlds(){
        return "file://"+QuickSetupHandler.getWorldsPath()
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
                    folder: quickSetup.getWorlds()
                    nameFilters: [ "*.sdf" ]
                }

                // model: folderModel
                // delegate: World {
                //   width: parent.width
                //   height: 100
                //   id: filePath
                //   // text: filePath
                //   text: fileName
                //   source: QuickSetupHandler.getThumbnail(fileURL, fileIsDir)
                //   onClicked: quickSetup.loadWorld(fileName, fileURL, fileIsDir)
                // }

              model: folderModel
              delegate: Button {
                width: parent.width
                height: 50
                text: fileName
                onClicked: {
                  quickSetup.loadWorld(fileName, fileURL, fileIsDir)
                }
                background: Rectangle {
                    color: fileIsDir ? "orange" : "gray"
                    border.color: "black"
                }
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
              id: checkBox
              text: "Don't show again"
              Layout.fillWidth: true
              checked: false
            //   onClicked: {
            //     QuickSetupHandler.onShowAgain(checkBox.checked);
            // }
          }
          Button {
            id: closeButton
            visible: true
            text: "Skip"
            Layout.alignment : Qt.AlignVCenter
            Layout.minimumWidth: width
            Layout.leftMargin: 10

            onClicked: {
              window.close();
              QuickSetupHandler.OnSkip();
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
}
