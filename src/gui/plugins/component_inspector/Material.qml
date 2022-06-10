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
  property double rAmbientItem
  property double gAmbientItem
  property double bAmbientItem
  property double aAmbientItem

  // Loaded items for diffuse red, green, blue, alpha
  property double rDiffuseItem
  property double gDiffuseItem
  property double bDiffuseItem
  property double aDiffuseItem

  // Loaded items for specular red, green, blue, alpha
  property double rSpecularItem
  property double gSpecularItem
  property double bSpecularItem
  property double aSpecularItem

  // Loaded items for emissive red, green, blue, alpha
  property double rEmissiveItem
  property double gEmissiveItem
  property double bEmissiveItem
  property double aEmissiveItem

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
        textVisible: false
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

        GridLayout {
          width: parent.width
          columns: 6

          // Ambient
          Item {
            Layout.row: 0
            Layout.columnSpan: 6
            Layout.fillWidth: true
            height: 70
            Loader {
              id: ambientLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
              onLoaded: {
                ambientLoader.item.r = model.data[0]
                ambientLoader.item.g = model.data[1]
                ambientLoader.item.b = model.data[2]
                ambientLoader.item.a = model.data[3] / 255.0
                rAmbientItem: ambientLoader.item.r
                gAmbientItem: ambientLoader.item.g
                bAmbientItem: ambientLoader.item.b
                aAmbientItem: ambientLoader.item.a
              }
            }
            Binding {
              target: ambientLoader.item
              property: "textVisible"
              value: true
            }
            Binding {
              target: ambientLoader.item
              property: "colorName"
              value: "Ambient"
            }
            Connections { 
              target : ambientLoader.item 
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            } 
          }

          // Diffuse
          Item {
            Layout.row: 1
            Layout.columnSpan: 6
            Layout.fillWidth: true
            height: 70
            Loader {
              id: diffuseLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
              onLoaded: {
                diffuseLoader.item.r = model.data[4]
                diffuseLoader.item.g = model.data[5]
                diffuseLoader.item.b = model.data[6]
                diffuseLoader.item.a = model.data[7] / 255.0
                rDiffuseItem: diffuseLoader.item.r
                gDiffuseItem: diffuseLoader.item.g
                bDiffuseItem: diffuseLoader.item.b
                aDiffuseItem: diffuseLoader.item.a
              }
            }
            Binding {
              target: diffuseLoader.item
              property: "colorName"
              value: "Diffuse  "
            }
            Connections { 
              target : diffuseLoader.item 
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            } 
          }

          // Specular
          Item {
            Layout.row: 2
            Layout.columnSpan: 6
            Layout.fillWidth: true
            height: 70
            Loader {
              id: specularLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
              onLoaded: {
                specularLoader.item.r = model.data[8]
                specularLoader.item.g = model.data[9]
                specularLoader.item.b = model.data[10]
                specularLoader.item.a = model.data[11] / 255.0
                rSpecularItem: specularLoader.item.r
                gSpecularItem: specularLoader.item.g
                bSpecularItem: specularLoader.item.b
                aSpecularItem: specularLoader.item.a
              }
            }
            Binding {
              target: specularLoader.item
              property: "colorName"
              value: "Specular"
            }
            Connections { 
              target : specularLoader.item 
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            } 
          }

          // Emissive
          Item {
            Layout.row: 3
            Layout.columnSpan: 6
            Layout.fillWidth: true
            height: 70
            Loader {
              id: emissiveLoader
              anchors.fill: parent
              sourceComponent: colorMaterial
              onLoaded: {
                emissiveLoader.item.r = model.data[12]
                emissiveLoader.item.g = model.data[13]
                emissiveLoader.item.b = model.data[14]
                emissiveLoader.item.a = model.data[15] / 255.0
                rEmissiveItem: emissiveLoader.item.r
                gEmissiveItem: emissiveLoader.item.g
                bEmissiveItem: emissiveLoader.item.b
                aEmissiveItem: emissiveLoader.item.a
              }
            }
            Binding {
              target: emissiveLoader.item
              property: "colorName"
              value: "Emissive"
            }
            Connections { 
              target : emissiveLoader.item 
              onColorSet: { 
                sendMaterialColor("", Qt.rgba(0, 0, 0, 0))
              }
            } 
          } // end Emissive
        } // end GridLayout
      } // end ColumnLayout (id: grid)
    } // Rectangle (id: content)
  }
}
