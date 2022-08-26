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

// Item displaying material color information
Rectangle {
  height: header.height + content.height
  width: componentInspectorEditor.width
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
  function sendMaterialColor(_type, _currColor) {
    componentInspectorEditor.onMaterialColor(
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
      aEmissiveItem.value,
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
    GzSpinBox {
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

          // rgba headers
          Text {
            text: "Red    "
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            Layout.row: 1
            Layout.column: 3
          }
          Text {
            text: "Green"
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            Layout.row: 1
            Layout.column: 4
          }
          Text {
            text: "Blue  "
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            Layout.row: 1
            Layout.column: 5
          }
          Text {
            text: "Alpha"
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            Layout.row: 1
            Layout.column: 6
          }

          // Ambient
          Text {
            text: " Ambient"
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            Layout.row: 2
            Layout.column: 1
          }
          // Ambient color dialog
          Button {
            id: ambientButton
            Layout.row: 2
            Layout.column: 2
            ToolTip.text: "Open color dialog"
            ToolTip.visible: hovered
            ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
            background: Rectangle {
              implicitWidth: 40
              implicitHeight: 40
              radius: 5
              border.color: getButtonColor(0, false)
              border.width: 2
              color: getButtonColor(0, true)
            }
            onClicked: {
              sendMaterialColor("ambient", getButtonColor(0, true))
            }
          }
          // Ambient red
          Item {
            Layout.row: 2
            Layout.column: 3
            Layout.fillWidth: true
            height: 40
            Loader {
              id: rAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[0]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                rAmbientItem = rAmbientLoader.item
              }
            }
          }
          // Ambient green
          Item {
            Layout.row: 2
            Layout.column: 4
            Layout.fillWidth: true
            height: 40
            Loader {
              id: gAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[1]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                gAmbientItem = gAmbientLoader.item
              }
            }
          }
          // Ambient blue
          Item {
            Layout.row: 2
            Layout.column: 5
            Layout.fillWidth: true
            height: 40
            Loader {
              id: bAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[2]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                bAmbientItem = bAmbientLoader.item
              }
            }
          }
          // Ambient alpha
          Item {
            Layout.row: 2
            Layout.column: 6
            Layout.fillWidth: true
            height: 40
            Loader {
              id: aAmbientLoader
              anchors.fill: parent
              property double numberValue: model.data[3]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                aAmbientItem = aAmbientLoader.item
              }
            }
          } // end Ambient

          // Diffuse
          Text {
            text: " Diffuse"
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            Layout.row: 3
            Layout.column: 1
          }
          // Diffuse color dialog
          Button {
            id: diffuseButton
            Layout.row: 3
            Layout.column: 2
            ToolTip.text: "Open color dialog"
            ToolTip.visible: hovered
            ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
            background: Rectangle {
              implicitWidth: 40
              implicitHeight: 40
              radius: 5
              border.color: getButtonColor(4, false)
              border.width: 2
              color: getButtonColor(4, true)
            }
            onClicked: {
              sendMaterialColor("diffuse", getButtonColor(4, true))
            }
          }
          // Diffuse red
          Item {
            Layout.row: 3
            Layout.column: 3
            Layout.fillWidth: true
            height: 40
            Loader {
              id: rDiffuseLoader
              anchors.fill: parent
              property double numberValue: model.data[4]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                rDiffuseItem = rDiffuseLoader.item
              }
            }
          }
          // Diffuse green
          Item {
            Layout.row: 3
            Layout.column: 4
            Layout.fillWidth: true
            height: 40
            Loader {
              id: gDiffuseLoader
              anchors.fill: parent
              property double numberValue: model.data[5]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                gDiffuseItem = gDiffuseLoader.item
              }
            }
          }
          // Diffuse blue
          Item {
            Layout.row: 3
            Layout.column: 5
            Layout.fillWidth: true
            height: 40
            Loader {
              id: bDiffuseLoader
              anchors.fill: parent
              property double numberValue: model.data[6]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                bDiffuseItem = bDiffuseLoader.item
              }
            }
          }
          // Diffuse alpha
          Item {
            Layout.row: 3
            Layout.column: 6
            Layout.fillWidth: true
            height: 40
            Loader {
              id: aDiffuseLoader
              anchors.fill: parent
              property double numberValue: model.data[7]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                aDiffuseItem = aDiffuseLoader.item
              }
            }
          } // end Diffuse

          // Specular
          Text {
            text: " Specular"
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            Layout.row: 4
            Layout.column: 1
          }
          // Specular color dialog
          Button {
            id: specularButton
            Layout.row: 4
            Layout.column: 2
            ToolTip.text: "Open color dialog"
            ToolTip.visible: hovered
            ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
            background: Rectangle {
              implicitWidth: 40
              implicitHeight: 40
              radius: 5
              border.color: getButtonColor(8, false)
              border.width: 2
              color: getButtonColor(8, true)
            }
            onClicked: {
              sendMaterialColor("specular", getButtonColor(8, true))
            }
          }
          // Specular red
          Item {
            Layout.row: 4
            Layout.column: 3
            Layout.fillWidth: true
            height: 40
            Loader {
              id: rSpecularLoader
              anchors.fill: parent
              property double numberValue: model.data[8]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                rSpecularItem = rSpecularLoader.item
              }
            }
          }
          // Specular green
          Item {
            Layout.row: 4
            Layout.column: 4
            Layout.fillWidth: true
            height: 40
            Loader {
              id: gSpecularLoader
              anchors.fill: parent
              property double numberValue: model.data[9]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                gSpecularItem = gSpecularLoader.item
              }
            }
          }
          // Specular blue
          Item {
            Layout.row: 4
            Layout.column: 5
            Layout.fillWidth: true
            height: 40
            Loader {
              id: bSpecularLoader
              anchors.fill: parent
              property double numberValue: model.data[10]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                bSpecularItem = bSpecularLoader.item
              }
            }
          }
          // Specular alpha
          Item {
            Layout.row: 4
            Layout.column: 6
            Layout.fillWidth: true
            height: 40
            Loader {
              id: aSpecularLoader
              anchors.fill: parent
              property double numberValue: model.data[11]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                aSpecularItem = aSpecularLoader.item
              }
            }
          } // end Specular

          // Emissive
          Text {
            text: " Emissive"
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            Layout.row: 5
            Layout.column: 1
          }
          // Emissive color dialog
          Button {
            id: emissiveButton
            Layout.row: 5
            Layout.column: 2
            ToolTip.text: "Open color dialog"
            ToolTip.visible: hovered
            ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
            background: Rectangle {
              implicitWidth: 40
              implicitHeight: 40
              radius: 5
              border.color: getButtonColor(12, false)
              border.width: 2
              color: getButtonColor(12, true)
            }
            onClicked: {
              sendMaterialColor("emissive", getButtonColor(12, true))
            }
          }
          // Emissive red
          Item {
            Layout.row: 5
            Layout.column: 3
            Layout.fillWidth: true
            height: 40
            Loader {
              id: rEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[12]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                rEmissiveItem = rEmissiveLoader.item
              }
            }
          }
          // Emissive green
          Item {
            Layout.row: 5
            Layout.column: 4
            Layout.fillWidth: true
            height: 40
            Loader {
              id: gEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[13]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                gEmissiveItem = gEmissiveLoader.item
              }
            }
          }
          // Emissive blue
          Item {
            Layout.row: 5
            Layout.column: 5
            Layout.fillWidth: true
            height: 40
            Loader {
              id: bEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[14]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                bEmissiveItem = bEmissiveLoader.item
              }
            }
          }
          // Emissive alpha
          Item {
            Layout.row: 5
            Layout.column: 6
            Layout.fillWidth: true
            height: 40
            Loader {
              id: aEmissiveLoader
              anchors.fill: parent
              property double numberValue: model.data[15]
              sourceComponent: spinBoxMaterialColor
              onLoaded: {
                aEmissiveItem = aEmissiveLoader.item
              }
            }
          } // end Emissive
        } // end GridLayout
      } // end ColumnLayout (id: grid)
    } // Rectangle (id: content)
  }
}
