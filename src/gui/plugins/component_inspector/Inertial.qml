/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

// TODO
// * IgnHelpers decimal
// * Reduce duplication on numbers

// Item displaying inertial information.
Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Light text color according to theme
  property string propertyColor: Material.theme == Material.Light ? "#444444" : "#bbbbbb"

  // Dark text color according to theme
  property string valueColor: Material.theme == Material.Light ? "black" : "white"

  // True if inertial has added mass
  property bool hasAddedMass: componentData.length === 34

  Column {
    anchors.fill: parent

    ExpandingTypeHeader {
      id: header
      // Using the default header text values.
    }

    // Content
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? column.height : 0
      clip: true
      color: "transparent"

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      Column {
        id: column
        width: parent.width - indentation - 2 * margin
        leftPadding: indentation + margin
        rightPadding: margin

        // Mass
        Row {
          width: parent.width
          height: massText.height

          Text {
            id : massText
            text: 'Mass (kg) '
            leftPadding: margin
            color: propertyColor
            font.pointSize: 12
          }
          Text {
            id: massValue
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: valueColor
            font.pointSize: 12
            text: {
              var decimals = getDecimals(massValue.width)
              var valueText = componentData ? componentData[6].toFixed(decimals) : "N/A"
              return valueText
            }
          }
        }

        // Pose
        Text {
          text: 'Pose'
          leftPadding: margin
          color: propertyColor
          font.pointSize: 12
        }
        // TODO pose

        // Inertia
        Text {
          text: 'Inertia'
          leftPadding: margin
          color: propertyColor
          font.pointSize: 12
        }

        GridLayout {
          width: parent.width
          columns: 6

          // Left spacer
          Item {
            Layout.rowSpan: 4
            width: margin + indentation
          }

          Item {
            width: margin
          }

          Text {
            text: 'x'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'y'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'z'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          // Right spacer
          Item {
            Layout.rowSpan: 4
            width: margin + indentation
          }

          Text {
            text: 'x'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[7]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[8]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[9]
          }

          Text {
            text: 'y'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[8]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[10]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[11]
          }

          Text {
            text: 'z'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[9]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[11]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[12]
          }
        }

        // Added mass
        Text {
          text: 'Fluid added mass'
          leftPadding: margin
          visible: hasAddedMass
          color: propertyColor
          font.pointSize: 12
        }

        GridLayout {
          width: parent.width
          columns: 7
          visible: hasAddedMass

          Item {
            width: margin
          }

          Text {
            text: 'x'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'y'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'z'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'p'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'q'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'r'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            text: 'x'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[13]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[14]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[15]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[16]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[17]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[18]
          }

          Text {
            text: 'y'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[14]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[19]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[20]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[21]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[22]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[23]
          }

          Text {
            text: 'z'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[15]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[20]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[24]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[25]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[26]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[27]
          }

          Text {
            text: 'p'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[16]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[21]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[25]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[28]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[29]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[30]
          }

          Text {
            text: 'q'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[17]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[22]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[26]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[29]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[31]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[32]
          }

          Text {
            text: 'r'
            horizontalAlignment: Text.AlignRight
            Layout.fillWidth: true
            color: propertyColor
            font.pointSize: 10
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[18]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[23]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[27]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[30]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[32]
          }

          Text {
            Layout.fillWidth: true
            horizontalAlignment: Text.AlignRight
            color: valueColor
            font.pointSize: 12
            text: componentData[33]
          }
        }
      }
    }
  }
}
