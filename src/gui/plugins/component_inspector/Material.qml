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
  property double rAmbientItem: model.data[0]
  property double gAmbientItem: model.data[1]
  property double bAmbientItem: model.data[2]
  property double aAmbientItem: model.data[3]

  // Loaded items for diffuse red, green, blue, alpha
  property double rDiffuseItem: model.data[4]
  property double gDiffuseItem: model.data[5]
  property double bDiffuseItem: model.data[6]
  property double aDiffuseItem: model.data[7]

  // Loaded items for specular red, green, blue, alpha
  property double rSpecularItem: model.data[8]
  property double gSpecularItem: model.data[9]
  property double bSpecularItem: model.data[10]
  property double aSpecularItem: model.data[11]

  // Loaded items for emissive red, green, blue, alpha
  property double rEmissiveItem: model.data[12]
  property double gEmissiveItem: model.data[13]
  property double bEmissiveItem: model.data[14]
  property double aEmissiveItem: model.data[15]

  // send new material color data to C++
  function sendMaterialColor() {
    componentInspector.onMaterialColor(
      rAmbientItem, gAmbientItem, bAmbientItem, aAmbientItem,
      rDiffuseItem, gDiffuseItem, bDiffuseItem, aDiffuseItem,
      rSpecularItem, gSpecularItem, bSpecularItem, aSpecularItem,
      rEmissiveItem, gEmissiveItem, bEmissiveItem, aEmissiveItem,
      "", Qt.rgba(0, 0, 0, 0)  // dummy placeholders (no longer needed)
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

  Component {
    id: gzcolor
    GzColor {
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
            Layout.topMargin: 10
            Layout.columnSpan: 6
            text: "      Color"
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
              sourceComponent: gzcolor
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
              value: model.data[3]
            }
            Connections {
              target: ambientLoader.item
              onGzColorSet: {
                rAmbientItem = ambientLoader.item.r
                gAmbientItem = ambientLoader.item.g
                bAmbientItem = ambientLoader.item.b
                aAmbientItem = ambientLoader.item.a
                sendMaterialColor()
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
              sourceComponent: gzcolor
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
              value: model.data[7]
            }
            Connections {
              target: diffuseLoader.item
              onGzColorSet: {
                rDiffuseItem = diffuseLoader.item.r
                gDiffuseItem = diffuseLoader.item.g
                bDiffuseItem = diffuseLoader.item.b
                aDiffuseItem = diffuseLoader.item.a
                sendMaterialColor()
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
              sourceComponent: gzcolor
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
              value: model.data[11]
            }
            Connections {
              target: specularLoader.item
              onGzColorSet: {
                rSpecularItem = specularLoader.item.r
                gSpecularItem = specularLoader.item.g
                bSpecularItem = specularLoader.item.b
                aSpecularItem = specularLoader.item.a
                sendMaterialColor()
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
              sourceComponent: gzcolor
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
              value: model.data[15]
            }
            Connections {
              target: emissiveLoader.item
              onGzColorSet: {
                rEmissiveItem = emissiveLoader.item.r
                gEmissiveItem = emissiveLoader.item.g
                bEmissiveItem = emissiveLoader.item.b
                aEmissiveItem = emissiveLoader.item.a
                sendMaterialColor()
              }
            }
          }
        } // end RowLayout
      } // end ColumnLayout (id: grid)
    } // Rectangle (id: content)
  }
}
