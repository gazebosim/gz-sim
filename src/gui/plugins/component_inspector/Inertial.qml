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
        columns: 8

        // Left spacer
        Item {
          Layout.rowSpan: 10
          width: margin + indentation
        }

        Text {
          id : massText
          text: 'Mass (kg)'
          leftPadding: 5
          color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
          font.pointSize: 12
        }
        Text {
          id: massValue
          Layout.fillWidth: true
          horizontalAlignment: Text.AlignRight
          color: Material.theme == Material.Light ? "black" : "white"
          font.pointSize: 12
          text: {
            var decimals = getDecimals(massValue.width)
            var valueText = componentData ? componentData[6].toFixed(decimals) : "N/A"
            return valueText
          }
        }

        // Right spacer
        Item {
          Layout.rowSpan: 10
          width: margin
        }
      }
    }
  }
}
