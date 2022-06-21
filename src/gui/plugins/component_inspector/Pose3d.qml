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
import "qrc:/qml"

// Item displaying 3D pose information.
Rectangle {
  height: header.height + gzPose.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Send new pose to C++
  function sendPose() {
    // TODO(anyone) There's a loss of precision when these values get to C++
    Pose3dImpl.OnPose(
      gzPose.xItem.value,
      gzPose.yItem.value,
      gzPose.zItem.value,
      gzPose.rollItem.value,
      gzPose.pitchItem.value,
      gzPose.yawItem.value
    );
  }

  Column {
    anchors.fill: parent

    // Header
    Rectangle {
      id: header
      width: parent.width
      height: typeHeader.height
      color: "transparent"

      RowLayout {
        anchors.fill: parent
        Item {
          width: margin
        }
        Image {
          id: icon
          sourceSize.height: indentation
          sourceSize.width: indentation
          fillMode: Image.Pad
          Layout.alignment : Qt.AlignVCenter
          source: gzPose.show ?
              "qrc:/Gazebo/images/minus.png" : "qrc:/Gazebo/images/plus.png"
        }
        TypeHeader {
          id: typeHeader
        }
        Item {
          Layout.fillWidth: true
        }
      }
      MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: {
          gzPose.show = !gzPose.show
          // console.log(model.data[0], model.data[1], model.data[2], model.data[3], model.data[4], model.data[5])
          // console.log(gzPose.xItem, gzPose.yItem, gzPose.zItem, gzPose.rollItem, gzPose.pitchItem, gzPose.yawItem)
        }
        onEntered: {
          header.color = highlightColor
        }
        onExited: {
          header.color = "transparent"
        }
      }
    }

    // Content
    GzPose {
      id: gzPose
      width: parent.width

      readOnly: {
        var isModel = entityType == "model"
        return !(isModel) || nestedModel
      }

      xModelValue: model.data[0]
      yModelValue: model.data[1]
      zModelValue: model.data[2]
      rollModelValue: model.data[3]
      pitchModelValue: model.data[4]
      yawModelValue: model.data[5]

      onPoseSet: {
        sendPose()
      }
    } // end gzPose
  } // end Column
} // end Rectangle
