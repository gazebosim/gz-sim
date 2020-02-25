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
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"

Rectangle {
  id: stringComponent
  height: header.height + content.height
  width: componentInspector.width
  color: "transparent"

  function textFromModel(_model) {
    if (_model && _model.data !== undefined && typeof _model.data == 'string')
      return _model.data

    return 'N/A'
  }

  Column {
    anchors.fill: parent

    Rectangle {
      id: header
      width: parent.width
      height: typeHeader.height
      color: "transparent"

      TypeHeader {
        id: typeHeader
        headerPadding: 0
      }
      MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: {
          content.show = !content.show
        }
        onEntered: {
          header.color = darkGrey
        }
        onExited: {
          header.color = "transparent"
        }
      }
    }

    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? textArea.height : 0
      clip: true
      color: darkGrey

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      TextArea {
        id: textArea
        width: parent.width
        text: textFromModel(model)
        color: Material.theme == Material.Light ? "black" : "white"
        font.pointSize: 12
        selectByMouse: true // will only work when we enable it
        enabled: false
      }
    }
  }
}
