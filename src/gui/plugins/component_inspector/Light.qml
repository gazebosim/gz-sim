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
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Maximum spinbox value
  property double spinMax: 1

  // Read-only / write
  property bool readOnly: {
    return false;
  }

  property int iconWidth: 20
  property int iconHeight: 20

  // Loaded item for R
  property var rSpecularItem: {}

  // Loaded item for G
  property var gSpecularItem: {}

  // Loaded item for B
  property var bSpecularItem: {}

  // Loaded item for A
  property var aSpecularItem: {}

  // Loaded item for R
  property var rDiffuseItem: {}

  // Loaded item for G
  property var gDiffuseItem: {}

  // Loaded item for B
  property var bDiffuseItem: {}

  // Loaded item for A
  property var aDiffuseItem: {}

  // Loaded item for R
  property var attRangeItem: {}

  // Loaded item for G
  property var attLinearItem: {}

  // Loaded item for B
  property var attConstantItem: {}

  // Loaded item for A
  property var attQuadraticItem: {}

  property var castShadowsItem: {}

  // Send new pose to C++
  function sendLight() {
    // TODO(anyone) There's a loss of precision when these values get to C++
    componentInspector.onLight(
      rSpecularItem.value,
      gSpecularItem.value,
      bSpecularItem.value,
      aSpecularItem.value,
      rDiffuseItem.value,
      gDiffuseItem.value,
      bDiffuseItem.value,
      aDiffuseItem.value,
      attRangeItem.value,
      attLinearItem.value,
      attConstantItem.value,
      attQuadraticItem.value,
      castShadowsItem
    );
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  /**
   * Used to create a spin box
   */
  Component {
    id: spinZeroOne
    IgnSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: 0
      maximumValue: spinMax
      decimals: 6
      onEditingFinished: {
        sendLight()
      }
    }
  }
  Component {
    id: spinZeroMax
    IgnSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: 0
      maximumValue: 1000000
      decimals: 6
      onEditingFinished: {
        sendLight()
      }
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
        var decimals = numberText.width < 100 ? 2 : 6
        return numberValue.toFixed(decimals)
      }
    }
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
          source: content.show ?
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
          content.show = !content.show
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
        columns: 6

        Text {
          Layout.columnSpan: 6
          text: "Specular"
          color: "dimgrey"
          width: margin + indentation
        }

        // Left spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
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

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: rSpecularText.width + indentation*3
          Loader {
            id: loaderSpecularR
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderSpecularR.item.componentInfo = "r"

          Text {
            id : rSpecularText
            text: ' R'
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
            id: rSpecularLoader
            anchors.fill: parent
            property double numberValue: model.data[0]
            sourceComponent: spinZeroOne
            onLoaded: {
              rSpecularItem = rSpecularLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: gSpecularText.width + indentation*3
          Loader {
            id: loaderSpecularG
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderSpecularG.item.componentInfo = "g"

          Text {
            id : gSpecularText
            text: ' G'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: gSpecularLoader
            anchors.fill: parent
            property double numberValue: model.data[1]
            sourceComponent: spinZeroOne
            onLoaded: {
              gSpecularItem = gSpecularLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: bSpecularText.width + indentation*3
          Loader {
            id: loaderSpecularB
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderSpecularB.item.componentInfo = "b"

          Text {
            id : bSpecularText
            text: ' B'
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
            id: bSpecularLoader
            anchors.fill: parent
            property double numberValue: model.data[2]
            sourceComponent: spinZeroOne
            onLoaded: {
              bSpecularItem = bSpecularLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: aSpecularText.width + indentation*3
          Loader {
            id: loaderSpecularA
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderSpecularA.item.componentInfo = "a"

          Text {
            id : aSpecularText
            text: ' A'
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
            id: aSpecularLoader
            anchors.fill: parent
            property double numberValue: model.data[3]
            sourceComponent: spinZeroOne
            onLoaded: {
              aSpecularItem = aSpecularLoader.item
            }
          }
        }
        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Text {
          Layout.columnSpan: 6
          text: "Diffuse"
          color: "dimgrey"
          width: margin + indentation
        }

        // Left spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: rDiffuseText.width + indentation*3
          Loader {
            id: loaderDiffuseR
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderDiffuseR.item.componentInfo = "r"

          Text {
            id : rDiffuseText
            text: ' R'
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
            id: rDiffuseLoader
            anchors.fill: parent
            property double numberValue: model.data[4]
            sourceComponent: spinZeroOne
            onLoaded: {
              rDiffuseItem = rDiffuseLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: gDiffuseText.width + indentation*3
          Loader {
            id: loaderDiffuseG
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderDiffuseG.item.componentInfo = "g"

          Text {
            id : gDiffuseText
            text: ' G'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: gDiffuseLoader
            anchors.fill: parent
            property double numberValue: model.data[5]
            sourceComponent: spinZeroOne
            onLoaded: {
              gDiffuseItem = gDiffuseLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: bDiffuseText.width + indentation*3
          Loader {
            id: loaderDiffuseB
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderDiffuseB.item.componentInfo = "b"

          Text {
            id : bDiffuseText
            text: ' B'
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
            id: bDiffuseLoader
            anchors.fill: parent
            property double numberValue: model.data[6]
            sourceComponent: spinZeroOne
            onLoaded: {
              bDiffuseItem = bDiffuseLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: aDiffuseText.width + indentation*3
          Loader {
            id: loaderDiffuseA
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderDiffuseA.item.componentInfo = "a"

          Text {
            id : aDiffuseText
            text: ' A'
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
            id: aDiffuseLoader
            anchors.fill: parent
            property double numberValue: model.data[7]
            sourceComponent: spinZeroOne
            onLoaded: {
              aDiffuseItem = aDiffuseLoader.item
            }
          }
        }

        Text {
          Layout.columnSpan: 6
          text: "Attenuation"
          color: "dimgrey"
          width: margin + indentation
        }

        // Left spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: attRangeText.width + indentation*3
          Loader {
            id: loaderAttRange
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderAttRange.item.componentInfo = "Attenuation Range"

          Text {
            id : attRangeText
            text: ' Range'
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
            id: attRangeLoader
            anchors.fill: parent
            property double numberValue: model.data[8]
            sourceComponent: spinZeroMax
            onLoaded: {
              attRangeItem = attRangeLoader.item
            }
          }
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: attLinearText.width + indentation*3
          Loader {
            id: loaderAttLinear
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderAttLinear.item.componentInfo = "Attenuation linear"

          Text {
            id : attLinearText
            text: ' Linear'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: attLinearLoader
            anchors.fill: parent
            property double numberValue: model.data[9]
            sourceComponent: spinZeroOne
            onLoaded: {
              attLinearItem = attLinearLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: attConstantText.width + indentation*3
          Loader {
            id: loaderAttConstant
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderAttConstant.item.componentInfo = "Attenuation  constant"

          Text {
            id : attConstantText
            text: ' Constant'
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
            id: attConstantLoader
            anchors.fill: parent
            property double numberValue: model.data[10]
            sourceComponent: spinZeroOne
            onLoaded: {
              attConstantItem = attConstantLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: attQuadraticText.width + indentation*3
          Loader {
            id: loaderAttQuadratic
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderAttQuadratic.item.componentInfo = "Attenuation quadratic"

          Text {
            id : attQuadraticText
            text: ' Quadratic'
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
            id: attQuadraticLoader
            anchors.fill: parent
            property double numberValue: model.data[11]
            sourceComponent: spinZeroMax
            onLoaded: {
              attQuadraticItem = attQuadraticLoader.item
            }
          }
        }

//////////////

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: castShadowsText.width + indentation*3
          Loader {
            id: loaderCastShadows
            width: iconWidth
            height: iconHeight
            y:10
            sourceComponent: plotIcon
          }
          Component.onCompleted: loaderCastShadows.item.componentInfo = "Cast shadows"

          Text {
            id : castShadowsText
            text: ' Cast shadows'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        Item {
          Layout.fillWidth: true
          height: 40

          Switch {
            id: booleanSwitchCastShadows
            checked: model.data[12]
            enabled: true

            onToggled: {
              castShadowsItem = checked
              sendLight()
            }
          }
        }
      }
    }
  }
}
