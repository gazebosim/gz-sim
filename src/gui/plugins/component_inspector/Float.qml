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

Rectangle {
  id: numberComponent
  height: typeHeader.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Horizontal margins
  property int margin: 5

  // Maximum spinbox value
  property double spinMax: 1000000

  RowLayout {
    anchors.fill: parent

    Item {
      height: parent.height
      width: margin
    }

    Item {
      height: parent.height
      width: indentation
    }

    TypeHeader {
      id: typeHeader
    }

    IgnSpinBox {
      id: content
      value: model.data
      minimumValue: -spinMax
      maximumValue: spinMax
      decimals: xSpin.width < 100 ? 2 : 6
      Layout.fillWidth: true
    }

    Item {
      height: parent.height
      width: margin
    }
  }
}
