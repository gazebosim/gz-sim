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
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying 3D pose information.
Rectangle {
  height: header.height + gzPoseInstance.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Send new pose to C++
  function sendPose(x, y, z, roll, pitch, yaw) {
    // TODO(anyone) There's a loss of precision when these values get to C++
    Pose3dImpl.OnPose(x, y, z, roll, pitch, yaw);
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
          source: gzPoseInstance.expand ?
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
          gzPoseInstance.expand = !gzPoseInstance.expand
        }
        onEntered: {
          header.color = highlightColor
        }
        onExited: {
          header.color = "transparent"
        }
      }
    }
    Rectangle {
      color: "transparent"
      width: parent.width
      height: gzPoseInstance.height

      GzPose {
        id: gzPoseInstance
        width: parent.width - margin * 2 - indentation
        x: margin + indentation
        readOnly: {
          var isModel = entityType == "model"
          return !(isModel) || nestedModel
        }

        xValue: model.data[0]
        yValue: model.data[1]
        zValue: model.data[2]
        rollValue: model.data[3]
        pitchValue: model.data[4]
        yawValue: model.data[5]

        onGzPoseSet: (_x, _y, _z, _roll, _pitch, _yaw) => {
          // _x, _y, _z, _roll, _pitch, _yaw are parameters of signal gzPoseSet
          // from gz-gui GzPose.qml
          sendPose(_x, _y, _z, _roll, _pitch, _yaw)
        }

        // By default pose widget is collapsed
        expand: false

        // plotting
        gzPlotEnabled: true
        gzPlotMimeDataX: { "text/plain" : (model === null) ? "" :
          "Component," + model.entity + "," + model.typeId + "," +
          model.dataType + ",x," + model.shortName
        }
        gzPlotMimeDataY: { "text/plain" : (model === null) ? "" :
          "Component," + model.entity + "," + model.typeId + "," +
          model.dataType + ",y," + model.shortName
        }
        gzPlotMimeDataZ: { "text/plain" : (model === null) ? "" :
          "Component," + model.entity + "," + model.typeId + "," +
          model.dataType + ",z," + model.shortName
        }
        gzPlotMimeDataRoll: { "text/plain" : (model === null) ? "" :
          "Component," + model.entity + "," + model.typeId + "," +
          model.dataType + ",roll," + model.shortName
        }
        gzPlotMimeDataPitch: { "text/plain" : (model === null) ? "" :
          "Component," + model.entity + "," + model.typeId + "," +
          model.dataType + ",pitch," + model.shortName
        }
        gzPlotMimeDataYaw: { "text/plain" : (model === null) ? "" :
          "Component," + model.entity + "," + model.typeId + "," +
          model.dataType + ",yaw," + model.shortName
        }
      } // end gzPoseInstance
    } // end Rectangle
  } // end Column
} // end Rectangle
