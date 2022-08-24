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
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspectorEditor"
import "qrc:/qml"

// Item displaying spherical coordinates information.
Rectangle {
  height: header.height + content.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Icon size
  property int iconWidth: 20
  property int iconHeight: 20

  // Send new data to C++
  function sendSphericalCoordinates() {
    // TODO(anyone) There's a loss of precision when these values get to C++
    componentInspectorEditor.onSphericalCoordinates(
      surfaceText.text,
      latSpin.value,
      lonSpin.value,
      elevationSpin.value,
      headingSpin.value
    );
  }

  Component {
    id: plotIcon
    Image {
      property string componentInfo: ""
      source: "plottable_icon.svg"
      anchors.top: parent.top
      anchors.left: parent.left

      Drag.mimeData: { "text/plain" : (model === null) ? "" :
      "Component," + model.entity + "," + model.typeId + "," +
                     model.dataType + "," + componentInfo + "," + model.shortName
      }
      Drag.dragType: Drag.Automatic
      Drag.supportedActions : Qt.CopyAction
      Drag.active: dragMouse.drag.active
      // a point to drag from
      Drag.hotSpot.x: 0
      Drag.hotSpot.y: y
      MouseArea {
        id: dragMouse
        anchors.fill: parent
        drag.target: (model === null) ? null : parent
        onPressed: parent.grabToImage(function(result) {parent.Drag.imageSource = result.url })
        onReleased: parent.Drag.drop();
        cursorShape: Qt.DragCopyCursor
      }
    }
  }

  Column {
    anchors.fill: parent

    // The expanding header. Make sure that the content to expand has an id set
    // to the value "content".
    ExpandingTypeHeader {
      id: header
      // Using the default header text values.
    }

    // Content
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? grid.height : 0
      clip: true
      color: "transparent"

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      GridLayout {
        id: grid
        width: parent.width
        columns: 4

        // Left spacer
        Item {
          Layout.rowSpan: 5
          width: margin + indentation
        }

        // Surface type
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: latText.width + indentation*3
          Rectangle {
            color: "transparent"
            id: surfaceSpacer
            height: iconHeight
            width: iconWidth
            y: 10
          }

          Text {
            text: 'Surface'
            leftPadding: 12
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.left: surfaceSpacer.right
            anchors.verticalCenter: parent.verticalCenter
          }
        }
        Text {
          id: surfaceText
          text: model.data[0]
          Layout.fillWidth: true
          horizontalAlignment: Text.AlignRight
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
          elide: Text.ElideLeft
        }

        // Right spacer
        Item {
          Layout.rowSpan: 5
          width: margin
        }

        // Latitude
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: latText.width + indentation*3
          Loader {
            id: loaderPlotLat
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderPlotLat.item.componentInfo = "latitude"

          Text {
            id : latText
            text: 'Latitude (°)'
            leftPadding: 15
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        GzSpinBox {
          id: latSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: model.data[1]
          value: latSpin.activeFocus ? latSpin.value : numberValue
          minimumValue: -90
          maximumValue: 90
          decimals: 12
          stepSize: 0.1
          onEditingFinished: {
            sendSphericalCoordinates()
          }
        }

        // Longitude
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: lonText.width + indentation*3
          Loader {
            id: loaderPlotLon
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderPlotLon.item.componentInfo = "longitude"

          Text {
            id : lonText
            text: 'Longitude (°)'
            leftPadding: 15
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        GzSpinBox {
          id: lonSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: model.data[2]
          value: lonSpin.activeFocus ? lonSpin.value : numberValue
          minimumValue: -180
          maximumValue: 180
          decimals: 12
          stepSize: 0.1
          onEditingFinished: {
            sendSphericalCoordinates()
          }
        }

        // Elevation
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: elevationText.width + indentation*3
          Loader {
            id: loaderPlotElevation
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderPlotElevation.item.componentInfo = "elevation"

          Text {
            id : elevationText
            text: 'Elevation (m)'
            leftPadding: 15
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        GzSpinBox {
          id: elevationSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: model.data[3]
          value: elevationSpin.activeFocus ? elevationSpin.value : numberValue
          minimumValue: -Number.MAX_VALUE
          maximumValue: Number.MAX_VALUE
          decimals: 12
          stepSize: 0.1
          onEditingFinished: {
            sendSphericalCoordinates()
          }
        }

        // Heading
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: headingText.width + indentation*3
          Loader {
            id: loaderPlotHeading
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderPlotHeading.item.componentInfo = "heading"

          Text {
            id : headingText
            text: 'Heading (°)'
            leftPadding: 15
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        GzSpinBox {
          id: headingSpin
          Layout.fillWidth: true
          height: 40
          property double numberValue: model.data[4]
          value: headingSpin.activeFocus ? headingSpin.value : numberValue
          minimumValue: -180
          maximumValue: 180
          decimals: 12
          stepSize: 0.1
          onEditingFinished: {
            sendSphericalCoordinates()
          }
        }
      }
    }
  }
}
