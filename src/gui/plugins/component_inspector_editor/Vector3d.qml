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
import "qrc:/ComponentInspectorEditor"
import "qrc:/qml"

// Item displaying 3D vector information.
Rectangle {
  height: header.height + content.height
  width: componentInspectorEditor.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // icon size
  property int iconWidth: 20
  property int iconHeight: 20

  // Horizontal margins
  property int margin: 5

  // Maximum spinbox value
  property double spinMax: Number.MAX_VALUE

  // Units, defaults to meters.
  property string unit: model && model.unit != undefined ? model.unit : 'm'

  // Readn-only / write
  property bool readOnly: true

  /**
   * Used to create a spin box
   */
  Component {
    id: writableNumber
    GzSpinBox {
      id: writableSpin
      value: numberValue
      minimumValue: -spinMax
      maximumValue: spinMax
      decimals: getDecimalsAdjustValue(writableSpin, numberValue)
    }
  }

  /**
   * Used to create a read-only number
   */
  Component {
    id: readOnlyNumber
    Text {
      id: numberText
      anchors.fill: parent
      horizontalAlignment: Text.AlignRight
      verticalAlignment: Text.AlignVCenter
      text: {
        var decimals = getDecimals(numberText.width)
        return numberValue.toFixed(decimals)
      }
    }
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
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: xText.width + indentation*3
          Loader {
            id: loaderX
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderX.item.componentInfo = "x"

          Text {
            id: xText
            text: ' X (' + unit + ')'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            anchors.fill: parent
            property double numberValue: model.data[0]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: xText.width + indentation*3
          Loader {
            id: loaderY
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderY.item.componentInfo = "y"

          Text {
            text: ' Y (' + unit + ')'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            anchors.fill: parent
            property double numberValue: model.data[1]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: xText.width + indentation*3
          Loader {
            id: loaderZ
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderZ.item.componentInfo = "z"

          Text {
            text: ' Z (' + unit + ')'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            anchors.fill: parent
            property double numberValue: model.data[2]
            sourceComponent: readOnly ? readOnlyNumber : writableNumber
          }
        }
      }
    }
  }
}
