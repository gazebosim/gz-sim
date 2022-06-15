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
  property double rAmbientItem: ambientLoader.item.r
  property double gAmbientItem: ambientLoader.item.g
  property double bAmbientItem: ambientLoader.item.b
  property double aAmbientItem: ambientLoader.item.a

  // Loaded items for diffuse red, green, blue, alpha
  property double rDiffuseItem: diffuseLoader.item.r
  property double gDiffuseItem: diffuseLoader.item.g
  property double bDiffuseItem: diffuseLoader.item.b
  property double aDiffuseItem: diffuseLoader.item.a

  // Loaded items for specular red, green, blue, alpha
  property double rSpecularItem: specularLoader.item.r
  property double gSpecularItem: specularLoader.item.g
  property double bSpecularItem: specularLoader.item.b
  property double aSpecularItem: specularLoader.item.a

  // Loaded items for emissive red, green, blue, alpha
  property double rEmissiveItem: emissiveLoader.item.r
  property double gEmissiveItem: emissiveLoader.item.g
  property double bEmissiveItem: emissiveLoader.item.b
  property double aEmissiveItem: emissiveLoader.item.a

  // send new material color data to C++
  function sendMaterialColor(_type, _currColor) {
    componentInspector.onMaterialColor(
      rAmbientItem,
      gAmbientItem,
      bAmbientItem,
      1.0 * aAmbientItem * 255.0,
      rDiffuseItem,
      gDiffuseItem,
      bDiffuseItem,
      1.0 * aDiffuseItem * 255.0,
      rSpecularItem,
      gSpecularItem,
      bSpecularItem,
      1.0 * aSpecularItem * 255.0,
      rEmissiveItem,
      gEmissiveItem,
      bEmissiveItem,
      1.0 * aEmissiveItem * 255.0,
      _type,
      _currColor
    );
  }

  // Get button color from model.data, related start indices
  // 0 = ambient
  // 4 = diffuse
  // 8 = specular
  // 12 = emissive
  function getButtonColor(_start_index, _isFillColor)
  {
    if (_isFillColor) {
      // fill color (of object in the scene)
      return Qt.rgba(model.data[_start_index],
                     model.data[_start_index + 1],
                     model.data[_start_index + 2],
                     model.data[_start_index + 3])
    } else {
      // border color's alpha set to 1 incase fill color is 0
      return Qt.rgba(model.data[_start_index],
                     model.data[_start_index + 1],
                     model.data[_start_index + 2],
                     1.0)
    }
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  // Used to create rgba spin boxes
  Component {
    id: spinBoxMaterialColor
    IgnSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: 0
      maximumValue: 255
      decimals: 0
      onEditingFinished: {
        // sending empty params to not open color dialog
        sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
      }
    }
  }

  Component {
    id: colorMaterial 
      GzColor { 
        id: gzcolor
        onColorSet: sendMaterialColor("", Qt.rgba(0, 0, 0, 0)) 
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
        spacing: 20


        RowLayout {
          // Color
          Text {
            Layout.columnSpan: 6
            text: "Color"
            color: "dimgrey"
            font.bold: true
          }
        }

        RowLayout {
          // Ambient
          Rectangle {
            color: "transparent"
            height: 50
            Layout.preferredWidth: ambientText.width + indentation*3

            Text {
              id : ambientText
              Layout.columnSpan: 2
              text: ' Ambient'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }

          // Ambient
          Item {
            Layout.fillWidth: true
            Layout.bottomMargin: 10
            height: 40
            Loader {
              id: ambientLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
            }
            Binding {
              target: ambientLoader.item
              property: "r"
              value: model.data[0]
            }
            Binding {
              target: ambientLoader.item
              property: "g"
              value: model.data[1]
            }
            Binding {
              target: ambientLoader.item
              property: "b"
              value: model.data[2]
            }
            Binding {
              target: ambientLoader.item
              property: "a"
              value: model.data[3] / 255.0
            }
            Connections { 
              target: ambientLoader.item
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            }
          }

          // Diffuse
          Rectangle {
            color: "transparent"
            height: 50
            Layout.preferredWidth: diffuseText.width + indentation*3

            Text {
              id : diffuseText
              Layout.columnSpan: 2
              text: ' Diffuse'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }

          // Diffuse
          Item {
            Layout.fillWidth: true
            Layout.bottomMargin: 10
            height: 40
            Loader {
              id: diffuseLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
            }
            Binding {
              target: diffuseLoader.item
              property: "r"
              value: model.data[4]
            }
            Binding {
              target: diffuseLoader.item
              property: "g"
              value: model.data[5]
            }
            Binding {
              target: diffuseLoader.item
              property: "b"
              value: model.data[6]
            }
            Binding {
              target: diffuseLoader.item
              property: "a"
              value: model.data[7] / 255.0
            }
            Connections { 
              target: diffuseLoader.item
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            }
          }
        }

        RowLayout {
          // Specular
          Rectangle {
            color: "transparent"
            height: 50
            Layout.preferredWidth: specularText.width + indentation*3

            Text {
              id : specularText
              Layout.columnSpan: 2
              text: ' Specular'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }

          // Specular
          Item {
            Layout.fillWidth: true
            Layout.bottomMargin: 10
            height: 40
            Loader {
              id: specularLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
            }
            Binding {
              target: specularLoader.item
              property: "r"
              value: model.data[8]
            }
            Binding {
              target: specularLoader.item
              property: "g"
              value: model.data[9]
            }
            Binding {
              target: specularLoader.item
              property: "b"
              value: model.data[10]
            }
            Binding {
              target: specularLoader.item
              property: "a"
              value: model.data[11] / 255.0
            }
            Connections { 
              target: specularLoader.item
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            }
          }

          // Emissive
          Rectangle {
            color: "transparent"
            height: 50
            Layout.preferredWidth: emissiveText.width + indentation*3

            Text {
              id : emissiveText
              Layout.columnSpan: 2
              text: 'Emissive'
              leftPadding: 5
              color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
              font.pointSize: 12
              anchors.centerIn: parent
            }
          }

          // Emissive
          Item {
            Layout.fillWidth: true
            Layout.bottomMargin: 10
            height: 40
            Loader {
              id: emissiveLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
            }
            Binding {
              target: emissiveLoader.item
              property: "r"
              value: model.data[12]
            }
            Binding {
              target: emissiveLoader.item
              property: "g"
              value: model.data[13]
            }
            Binding {
              target: emissiveLoader.item
              property: "b"
              value: model.data[14]
            }
            Binding {
              target: emissiveLoader.item
              property: "a"
              value: model.data[15] / 255.0
            }
            Connections { 
              target: emissiveLoader.item
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            }
          }
        } // end RowLayout
      } // end ColumnLayout (id: grid)
    } // Rectangle (id: content)
  }
}
