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
import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: insertmodel
  color: "transparent"
  Layout.minimumWidth: 200
  Layout.minimumHeight: 500
  anchors.fill: parent
  GridView {
    id: gridView
    anchors.fill: parent

    model: LocalModelList
    delegate: Rectangle {
      width: gridView.cellWidth
      height: gridView.cellHeight
      color: "transparent"
      Image {
        anchors.fill: parent
        anchors.margins: 1
        source: model.thumbnail
        fillMode: Image.PreserveAspectFit
      }
      MouseArea {
        anchors.fill: parent
        onClicked: {
          print(model.name)
          InsertModel.OnMode(model.sdf);
        }
      }
      Text {
        text: model.name
        width: parent.width
        height: parent.height
        minimumPointSize: 5
        font.pointSize: 60
        fontSizeMode: Text.Fit
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
      }
   }
 }
}
