/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
import "qrc:/ComponentInspectorEditor"
import "qrc:/qml"

// Item displaying joint type information.
Rectangle {
  id: jointTypeComponent
  height: typeHeader.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  ListModel {
    id: jointTypes
    ListElement { type: "Ball" }
    ListElement { type: "Continuous" }
    ListElement { type: "Fixed" }
    ListElement { type: "Gearbox" }
    ListElement { type: "Prismatic" }
    ListElement { type: "Revolute" }
    ListElement { type: "Revolute2" }
    ListElement { type: "Screw" }
    ListElement { type: "Universal" }
  }

  function indexFromModel(_model) {
    if (_model && _model.data !== undefined)
    {
      for (var i = 0; i < jointTypes.count; i++)
      {
        if (jointTypes.get(i).type === _model.data)
        {
          return i
        }
      }
    }
    return -1
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  RowLayout {
    anchors.fill: parent

    Item {
      height: parent.height
      width: margin
    }

    Item {
      height: parent.height
      width: indentation
    }

    TypeHeader {
      id: typeHeader
    }

    Item {
      height: parent.height
      Layout.fillWidth: true
    }

    QtObject{
      // Workaround to keep from using the ListModel as model
      id: indexObj
      property int index: indexFromModel(model)
    }

    ComboBox {
      id: jointType
      padding: 0
      textRole: "type"
      model: jointTypes
      currentIndex: indexObj.index
      Layout.alignment: Qt.AlignRight
      font.pointSize: 12
      background: Rectangle {
        color: "transparent"
        implicitWidth: 140
        implicitHeight: typeHeader.height
      }
      enabled: componentInspectorEditor.getSimPaused()
      onActivated: {
        JointTypeImpl.OnJointType(currentText)
      }
    }

    Item {
      height: parent.height
      width: margin
    }
  }
}
