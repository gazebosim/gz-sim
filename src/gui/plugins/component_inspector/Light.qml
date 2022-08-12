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
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying light information.
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

  // Loaded item for specular red
  property var rSpecularItem: {}

  // Loaded item for specular green
  property var gSpecularItem: {}

  // Loaded item for specular blue
  property var bSpecularItem: {}

  // Loaded item for specular alpha
  property var aSpecularItem: {}

  // Loaded item for diffuse red
  property var rDiffuseItem: {}

  // Loaded item for diffuse green
  property var gDiffuseItem: {}

  // Loaded item for diffuse blue
  property var bDiffuseItem: {}

  // Loaded item for diffuse alpha
  property var aDiffuseItem: {}

  // Loaded item for attenuation range
  property var attRangeItem: {}

  // Loaded item for attenuation linear
  property var attLinearItem: {}

  // Loaded item for attenuation constant
  property var attConstantItem: {}

  // Loaded item for attenuation quadratic
  property var attQuadraticItem: {}

  // Loaded item for cast shadows
  property var castShadowsItem: {}

  // Loaded item for direction X (spotlight or directional)
  property var directionXItem: {}

  // Loaded item for direction Y (spotlight or directional)
  property var directionYItem: {}

  // Loaded item for direction Z (spotlight or directional)
  property var directionZItem: {}

  // Loaded item for inner angle (spotlight)
  property var innerAngleItem: {}

  // Loaded item for inner angle (spotlight)
  property var outerAngleItem: {}

  // Loaded item for falloff (spotlight)
  property var falloffItem: {}

  // Loaded item for intensity
  property var intensityItem: {}

  // Loaded item for isLightOn
  property var isLightOnItem: {}

  // Loaded item for visualizeVisuals
  property var visualizeVisualItem: {}

  // Send new light data to C++
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
      castShadowsItem.checked,
      directionXItem.value,
      directionYItem.value,
      directionZItem.value,
      innerAngleItem.value,
      outerAngleItem.value,
      falloffItem.value,
      intensityItem.value,
      model.data[20],
      isLightOnItem.checked,
      visualizeVisualItem.checked
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
    id: sliderZeroOne
    Slider {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      from: 0.0
      to: 1.0
      onValueChanged: {
        if (hovered){
          sendLight()
        }
      }
    }
  }

  Component {
    id: ignSwitch
    Switch {
      id: booleanSwitch
      checked: numberValue
      enabled: true
      onToggled: {
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
  Component {
    id: spinNoLimit
    IgnSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: -100000
      maximumValue: 100000
      decimals: 6
      onEditingFinished: {
        sendLight()
      }
    }
  }

  Component {
    id: gzPlotIcon
    GzPlotIcon {
      gzMimeData: { "text/plain" : (model === null) ? "" :
        "Component," + model.entity + "," + model.typeId + "," +
        model.dataType + "," + gzComponentInfo + "," + model.shortName
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

      ColumnLayout {
        id: grid
        width: parent.width

        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: visualizeVisualText.width + indentation*3

            Text {
              id : visualizeVisualText
              text: ' View gizmo'
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
              id: visualizeVisualLoader
              anchors.fill: parent
              property double numberValue: model.data[22]
              sourceComponent: ignSwitch
              onLoaded: {
                visualizeVisualItem = visualizeVisualLoader.item
              }
            }
          }
        }

        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: isOnText.width + indentation*3

            Text {
              id : isOnText
              text: ' Turn on/off'
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
              id: isOnLoader
              anchors.fill: parent
              property double numberValue: model.data[21]
              sourceComponent: ignSwitch
              onLoaded: {
                isLightOnItem = isOnLoader.item
              }
            }
          }
        }

        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: intensityText.width + indentation*3
            Loader {
              id: loaderIntensity
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "intensity"
            }

            Text {
              id : intensityText
              text: ' Intensity:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: intensityLoader
              anchors.fill: parent
              property double numberValue: model.data[19]
              sourceComponent: spinZeroMax
              onLoaded: {
                intensityItem = intensityLoader.item
              }
            }
          }
        }
        RowLayout {
          Layout.alignment : Qt.AlignLeft
          Text {
            text: "      Specular"
            color: "dimgrey"
            width: margin + indentation
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: rSpecularText.width + indentation*3
            Loader {
              id: loaderSpecularR
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "specularR"
            }

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
              sourceComponent: sliderZeroOne
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
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "specularG"
            }

            Text {
              id : gSpecularText
              text: ' G'
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
              property double numberValue: model.data[1]
              sourceComponent: sliderZeroOne
              onLoaded: {
                gSpecularItem = gSpecularLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: bSpecularText.width + indentation*3
            Loader {
              id: loaderSpecularB
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "specularB"
            }

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
              sourceComponent: sliderZeroOne
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
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "specularA"
            }

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
              sourceComponent: sliderZeroOne
              onLoaded: {
                aSpecularItem = aSpecularLoader.item
              }
            }
          }
        }
        RowLayout {
          Layout.alignment: Qt.AlignHCenter
          Button {
            Layout.alignment: Qt.AlignHCenter
            id: specularColor
            text: qsTr("Specular Color")
            onClicked: colorDialog.open()
            ColorDialog {
              id: colorDialog
              title: "Choose a specular color"
              visible: false
              onAccepted: {
                rSpecularLoader.item.value = colorDialog.color.r
                gSpecularLoader.item.value = colorDialog.color.g
                bSpecularLoader.item.value = colorDialog.color.b
                aSpecularLoader.item.value = colorDialog.color.a
                sendLight()
                colorDialog.close()
              }
              onRejected: {
                colorDialog.close()
              }
            }
          }
        }
        RowLayout {
          Text {
            Layout.columnSpan: 6
            text: "      Diffuse"
            color: "dimgrey"
            width: margin + indentation
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: rDiffuseText.width + indentation*3
            Loader {
              id: loaderDiffuseR
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "diffuseR"
            }

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
              sourceComponent: sliderZeroOne
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
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "diffuseG"
            }

            Text {
              id : gDiffuseText
              text: ' G'
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
              sourceComponent: sliderZeroOne
              onLoaded: {
                gDiffuseItem = gDiffuseLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: bDiffuseText.width + indentation*3
            Loader {
              id: loaderDiffuseB
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "diffuseB"
            }

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
              sourceComponent: sliderZeroOne
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
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "diffuseA"
            }

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
              sourceComponent: sliderZeroOne
              onLoaded: {
                aDiffuseItem = aDiffuseLoader.item
              }
            }
          }
        }
        RowLayout {
          Layout.alignment: Qt.AlignHCenter
          Button {
            Layout.alignment: Qt.AlignHCenter
            id: diffuseColor
            text: qsTr("Diffuse Color")
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
                sendLight()
                colorDialogDiffuse.close()
              }
              onRejected: {
                colorDialogDiffuse.close()
              }
            }
          }
        }
        RowLayout {
          Text {
            Layout.columnSpan: 6
            text: "      Attenuation"
            color: "dimgrey"
            width: margin + indentation
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: attRangeText.width + indentation*3
            Loader {
              id: loaderAttRange
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "attRange"
            }

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
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: attLinearText.width + indentation*3
            Loader {
              id: loaderAttLinear
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "attLinear"
            }

            Text {
              id : attLinearText
              text: ' Linear'
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
              id: attLinearLoader
              anchors.fill: parent
              property double numberValue: model.data[9]
              sourceComponent: sliderZeroOne
              onLoaded: {
                attLinearItem = attLinearLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: attConstantText.width + indentation*3
            Loader {
              id: loaderAttConstant
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "attConstant"
            }

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
              sourceComponent: sliderZeroOne
              onLoaded: {
                attConstantItem = attConstantLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: attQuadraticText.width + indentation*3
            Loader {
              id: loaderAttQuadratic
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "attQuadratic"
            }

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
        }
        RowLayout {
          Rectangle {
            color: "transparent"
            height: 40
            Layout.preferredWidth: castShadowsText.width + indentation*3
            Loader {
              id: loaderCastShadows
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "castshadows"
            }

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

            Loader {
              id: castShadowLoader
              anchors.fill: parent
              property double numberValue: model.data[12]
              sourceComponent: ignSwitch
              onLoaded: {
                castShadowsItem = castShadowLoader.item
              }
            }
          }
        }
        RowLayout {
          Text {
            visible: model.data[20] === 1 || model.data[20] === 2
            Layout.columnSpan: 6
            text: "      Direction"
            color: "dimgrey"
            width: margin + indentation
          }
        }
        RowLayout {
          Rectangle {
            visible: model.data[20] === 1 || model.data[20] === 2
            color: "transparent"
            height: 40
            Layout.preferredWidth: xDirectionText.width + indentation*3
            Loader {
              id: loaderDirectionX
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "directionX"
            }

            Text {
              visible: model.data[20] === 1 || model.data[20] === 2
              id : xDirectionText
              text: ' X:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            visible: model.data[20] === 1 || model.data[20] === 2
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: xDirectionLoader
              anchors.fill: parent
              property double numberValue: model.data[13]
              sourceComponent: spinNoLimit
              onLoaded: {
                directionXItem = xDirectionLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            visible: model.data[20] === 1 || model.data[20] === 2
            color: "transparent"
            height: 40
            Layout.preferredWidth: yDirectionText.width + indentation*3
            Loader {
              id: loaderDirectionY
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "directionY"
            }

            Text {
              visible: model.data[20] === 1 || model.data[20] === 2
              id : yDirectionText
              text: ' Y:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            visible: model.data[20] === 1 || model.data[20] === 2
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: yDirectionLoader
              anchors.fill: parent
              property double numberValue: model.data[14]
              sourceComponent: spinNoLimit
              onLoaded: {
                directionYItem = yDirectionLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            visible: model.data[20] === 1 || model.data[20] === 2
            color: "transparent"
            height: 40
            Layout.preferredWidth: zDirectionText.width + indentation*3
            Loader {
              id: loaderDirectionZ
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "directionZ"
            }

            Text {
              visible: model.data[20] === 1 || model.data[20] === 2
              id : zDirectionText
              text: ' Z:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            visible: model.data[20] === 1 || model.data[20] === 2
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: zDirectionLoader
              anchors.fill: parent
              property double numberValue: model.data[15]
              sourceComponent: spinNoLimit
              onLoaded: {
                directionZItem = zDirectionLoader.item
              }
            }
          }
        }
        RowLayout {
          Text {
            visible: model.data[20] === 1
            Layout.columnSpan: 6
            text: "      Spot features"
            color: "dimgrey"
            width: margin + indentation
          }
        }
        RowLayout {
          Rectangle {
            visible: model.data[20] === 1
            color: "transparent"
            height: 40
            Layout.preferredWidth: innerAngleText.width + indentation*3
            Loader {
              id: loaderInnerAngle
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "innerAngle"
            }

            Text {
              visible: model.data[20] === 1
              id : innerAngleText
              text: ' Inner Angle:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            visible: model.data[20] === 1
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: innerAngleLoader
              anchors.fill: parent
              property double numberValue: model.data[16]
              sourceComponent: spinNoLimit
              onLoaded: {
                innerAngleItem = innerAngleLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            visible: model.data[20] === 1
            color: "transparent"
            height: 40
            Layout.preferredWidth: outerAngleText.width + indentation*3
            Loader {
              id: loaderOuterAngle
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "outerAngle"
            }

            Text {
              visible: model.data[20] === 1
              id : outerAngleText
              text: ' Outer angle:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            visible: model.data[20] === 1
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: outerAngleLoader
              anchors.fill: parent
              property double numberValue: model.data[17]
              sourceComponent: spinNoLimit
              onLoaded: {
                outerAngleItem = outerAngleLoader.item
              }
            }
          }
        }
        RowLayout {
          Rectangle {
            visible: model.data[20] === 1
            color: "transparent"
            height: 40
            Layout.preferredWidth: fallOffText.width + indentation*3
            Loader {
              id: loaderFallOff
              sourceComponent: gzPlotIcon
              property string gzComponentInfo: "falloff"
            }

            Text {
              visible: model.data[20] === 1
              id : fallOffText
              text: ' Falloff:'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }
          Item {
            visible: model.data[20] === 1
            Layout.fillWidth: true
            height: 40
            Layout.columnSpan: 4
            Loader {
              id: fallOffLoader
              anchors.fill: parent
              property double numberValue: model.data[18]
              sourceComponent: spinZeroMax
              onLoaded: {
                falloffItem = fallOffLoader.item
              }
            }
          }
        }
      }
    }
  }
}
