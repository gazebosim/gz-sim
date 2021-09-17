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
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying material color information
Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  property int iconWidth: 20
  property int iconHeight: 20

  // Loaded items for ambient red, green, blue, alpha
  property var rAmbientItem: {}
  property var gAmbientItem: {}
  property var bAmbientItem: {}
  property var aAmbientItem: {}

  // Loaded items for diffuse red, green, blue, alpha
  property var rDiffuseItem: {}
  property var gDiffuseItem: {}
  property var bDiffuseItem: {}
  property var aDiffuseItem: {}

  // Loaded items for specular red, green, blue, alpha
  property var rSpecularItem: {}
  property var gSpecularItem: {}
  property var bSpecularItem: {}
  property var aSpecularItem: {}

  // Loaded items for emissive red, green, blue, alpha
  property var rEmissiveItem: {}
  property var gEmissiveItem: {}
  property var bEmissiveItem: {}
  property var aEmissiveItem: {}

  // send new material color data to C++
  function sendMaterialColor() {
    // TODO(anyone) There's a loss of precision when these values get to C++
    componentInspector.onMaterialColor(
      rAmbientItem.value,
      gAmbientItem.value,
      bAmbientItem.value,
      aAmbientItem.value,
      rDiffuseItem.value,
      gDiffuseItem.value,
      bDiffuseItem.value,
      aDiffuseItem.value,
      rSpecularItem.value,
      gSpecularItem.value,
      bSpecularItem.value,
      aSpecularItem.value,
      rEmissiveItem.value,
      gEmissiveItem.value,
      bEmissiveItem.value,
      aEmissiveItem.value
    );
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  // Used to create rgba sliders
  Component {
    id: sliderMaterialColor
    Slider {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      from: 0.0
      to: 1.0
      onValueChanged: {
        if (hovered) {
          sendMaterialColor()
        }
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
          duration: 200
          easing.type: Easing.InOutQuad
        }
      }

      ColumnLayout {
        id: grid
        width: parent.width

        // Ambient
        RowLayout {
          Layout.alignment : Qt.AlignLeft
          Text {
            text: "      Ambient"
            font.weight: Font.Bold
            color: "dimgrey"
            width: margin + indentation
          }
        }
        // Ambient rgba text & sliders
        RowLayout {
          // Ambient red
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: rAmbientText.width + indentation*3
            Loader {
              id: loaderAmbientR
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderAmbientR.item.componentInfo = "ambientR"

            Text {
              id: rAmbientText
              text: " R"
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
              id: rAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[0]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                rAmbientItem = rAmbientLoader.item
              }
            }
          }
          // Ambient green
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: gAmbientText.width + indentation*3
            Loader {
              id: loaderAmbientG
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderAmbientG.item.componentInfo = "ambientG"

            Text {
              id: gAmbientText
              text: " G"
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
              id: gAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[1]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                gAmbientItem = gAmbientLoader.item
              }
            }
          }
        }
        RowLayout {
          // Ambient blue
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: bAmbientText.width + indentation*3
            Loader {
              id: loaderAmbientB
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderAmbientB.item.componentInfo = "ambientB"

            Text {
              id: bAmbientText
              text: " B"
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
              id: bAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[2]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                bAmbientItem = bAmbientLoader.item
              }
            }
          }
          // Ambient alpha
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: aAmbientText.width + indentation*3
            Loader {
              id: loaderAmbientA
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderAmbientA.item.componentInfo = "ambientA"

            Text {
              id: aAmbientText
              text: " A"
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
              id: aAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[3]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                aAmbientItem = aAmbientLoader.item
              }
            }
          }
        }
        // Ambient color dialog button
        RowLayout {
          Layout.alignment: Qt.AlignHCenter
          Button {
            Layout.alignment: Qt.AlignHCenter
            id: colorButtonAmbient
            text: "Ambient Color"
            onClicked: colorDialogAmbient.open()
            ColorDialog {
              id: colorDialogAmbient
              title: "Choose an ambient color"
              visible: false
              onAccepted: {
                rAmbientLoader.item.value = colorDialogAmbient.color.r
                gAmbientLoader.item.value = colorDialogAmbient.color.g
                bAmbientLoader.item.value = colorDialogAmbient.color.b
                aAmbientLoader.item.value = colorDialogAmbient.color.a
                sendMaterialColor()
                colorDialogAmbient.close()
              }
              onRejected: {
                colorDialogAmbient.close()
              }
            }
          }
        }
        // end Ambient

        // Diffuse
        RowLayout {
          Layout.alignment : Qt.AlignLeft
          Text {
            text: "      Diffuse"
            font.weight: Font.Bold
            color: "dimgrey"
            width: margin + indentation
          }
        }
        // Diffuse rgba text & sliders
        RowLayout {
          // Diffuse red
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: rDiffuseText.width + indentation*3
            Loader {
              id: loaderDiffuseR
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderDiffuseR.item.componentInfo = "DiffuseR"

            Text {
              id: rDiffuseText
              text: " R"
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
              sourceComponent: sliderMaterialColor
              onLoaded: {
                rDiffuseItem = rDiffuseLoader.item
              }
            }
          }
          // Diffuse green
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: gDiffuseText.width + indentation*3
            Loader {
              id: loaderDiffuseG
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderDiffuseG.item.componentInfo = "diffuseG"

            Text {
              id: gDiffuseText
              text: " G"
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
              id: gDiffuseLoader
              anchors.fill: parent
              property double numberValue: model.data[5]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                gDiffuseItem = gDiffuseLoader.item
              }
            }
          }
        }
        RowLayout {
          // Diffuse blue
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: bDiffuseText.width + indentation*3
            Loader {
              id: loaderDiffuseB
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderDiffuseB.item.componentInfo = "diffuseB"

            Text {
              id: bDiffuseText
              text: " B"
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
              sourceComponent: sliderMaterialColor
              onLoaded: {
                bDiffuseItem = bDiffuseLoader.item
              }
            }
          }
          // Diffuse alpha
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: aDiffuseText.width + indentation*3
            Loader {
              id: loaderDiffuseA
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderDiffuseA.item.componentInfo = "diffuseA"

            Text {
              id: aDiffuseText
              text: " A"
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
              sourceComponent: sliderMaterialColor
              onLoaded: {
                aDiffuseItem = aDiffuseLoader.item
              }
            }
          }
        }
        // Diffuse color dialog button
        RowLayout {
          Layout.alignment: Qt.AlignHCenter
          Button {
            Layout.alignment: Qt.AlignHCenter
            id: colorButtonDiffuse
            text: "Diffuse Color"
            onClicked: colorDialogDiffuse.open()
            ColorDialog {
              id: colorDialogDiffuse
              title: "Choose a diffuse color"
              visible: false
              onAccepted: {
                rDiffuseLoader.item.value = colorDialogDiffuse.color.r
                gDiffuseLoader.item.value = colorDialogDiffuse.color.g
                bDiffuseLoader.item.value = colorDialogDiffuse.color.b
                aDiffuseLoader.item.value = colorDialogDiffuse.color.a
                sendMaterialColor()
                colorDialogDiffuse.close()
              }
              onRejected: {
                colorDialogDiffuse.close()
              }
            }
          }
        }
        // end Diffuse


        // Specular
        RowLayout {
          Layout.alignment : Qt.AlignLeft
          Text {
            text: "      Specular"
            font.weight: Font.Bold
            color: "dimgrey"
            width: margin + indentation
          }
        }
        // Specular rgba text & sliders
        RowLayout {
          // Specular red
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: rSpecularText.width + indentation*3
            Loader {
              id: loaderSpecularR
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderSpecularR.item.componentInfo = "specularR"

            Text {
              id: rSpecularText
              text: " R"
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
              property double numberValue: model.data[8]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                rSpecularItem = rSpecularLoader.item
              }
            }
          }
          // Specular green
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: gSpecularText.width + indentation*3
            Loader {
              id: loaderSpecularG
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderSpecularG.item.componentInfo = "specularG"

            Text {
              id: gSpecularText
              text: " G"
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
              id: gSpecularLoader
              anchors.fill: parent
              property double numberValue: model.data[9]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                gSpecularItem = gSpecularLoader.item
              }
            }
          }
        }
        RowLayout {
          // Specular blue
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: bSpecularText.width + indentation*3
            Loader {
              id: loaderSpecularB
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderSpecularB.item.componentInfo = "specularB"

            Text {
              id: bSpecularText
              text: " B"
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
              property double numberValue: model.data[10]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                bSpecularItem = bSpecularLoader.item
              }
            }
          }
          // Specular alpha
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: aSpecularText.width + indentation*3
            Loader {
              id: loaderSpecularA
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderSpecularA.item.componentInfo = "specularA"

            Text {
              id: aSpecularText
              text: " A"
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
              property double numberValue: model.data[11]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                aSpecularItem = aSpecularLoader.item
              }
            }
          }
        }
        // Specular color dialog button
        RowLayout {
          Layout.alignment: Qt.AlignHCenter
          Button {
            Layout.alignment: Qt.AlignHCenter
            id: colorButtonSpecular
            text: "Specular Color"
            onClicked: colorDialogSpecular.open()
            ColorDialog {
              id: colorDialogSpecular
              title: "Choose a specular color"
              visible: false
              onAccepted: {
                rSpecularLoader.item.value = colorDialogSpecular.color.r
                gSpecularLoader.item.value = colorDialogSpecular.color.g
                bSpecularLoader.item.value = colorDialogSpecular.color.b
                aSpecularLoader.item.value = colorDialogSpecular.color.a
                sendMaterialColor()
                colorDialogSpecular.close()
              }
              onRejected: {
                colorDialogSpecular.close()
              }
            }
          }
        }
        // end Specular

        // Emissive
        RowLayout {
          Layout.alignment : Qt.AlignLeft
          Text {
            text: "      Emissive"
            font.weight: Font.Bold
            color: "dimgrey"
            width: margin + indentation
          }
        }
        // Emissive rgba text & sliders
        RowLayout {
          // Emissive red
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: rEmissiveText.width + indentation*3
            Loader {
              id: loaderEmissiveR
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderEmissiveR.item.componentInfo = "emissiveR"

            Text {
              id: rEmissiveText
              text: " R"
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
              id: rEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[12]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                rEmissiveItem = rEmissiveLoader.item
              }
            }
          }
          // Emissive green
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: gEmissiveText.width + indentation*3
            Loader {
              id: loaderEmissiveG
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderEmissiveG.item.componentInfo = "emissiveG"

            Text {
              id: gEmissiveText
              text: " G"
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
              id: gEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[13]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                gEmissiveItem = gEmissiveLoader.item
              }
            }
          }
        }
        RowLayout {
          // Emissive blue
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: bEmissiveText.width + indentation*3
            Loader {
              id: loaderEmissiveB
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderEmissiveB.item.componentInfo = "emissiveB"

            Text {
              id: bEmissiveText
              text: " B"
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
              id: bEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[14]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                bEmissiveItem = bEmissiveLoader.item
              }
            }
          }
          // Emissive alpha
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: aEmissiveText.width + indentation*3
            Loader {
              id: loaderEmissiveA
              width: iconWidth
              height: iconHeight
              y: 10
              sourceComponent: plotIcon
            }
            Component.onCompleted: loaderEmissiveA.item.componentInfo = "emissiveA"

            Text {
              id: aEmissiveText
              text: " A"
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
              id: aEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[15]
              sourceComponent: sliderMaterialColor
              onLoaded: {
                aEmissiveItem = aEmissiveLoader.item
              }
            }
          }
        }
        // Emissive color dialog button
        RowLayout {
          Layout.alignment: Qt.AlignHCenter
          Button {
            Layout.alignment: Qt.AlignHCenter
            id: colorButtonEmissive
            text: "Emissive Color"
            onClicked: colorDialogEmissive.open()
            ColorDialog {
              id: colorDialogEmissive
              title: "Choose a emissive color"
              visible: false
              onAccepted: {
                rEmissiveLoader.item.value = colorDialogEmissive.color.r
                gEmissiveLoader.item.value = colorDialogEmissive.color.g
                bEmissiveLoader.item.value = colorDialogEmissive.color.b
                aEmissiveLoader.item.value = colorDialogEmissive.color.a
                sendMaterialColor()
                colorDialogEmissive.close()
              }
              onRejected: {
                colorDialogEmissive.close()
              }
            }
          }
        }
        // end Emissive

      } // ColumnLayout (id: grid)
    } // Rectangle (id: content)
  }
}
