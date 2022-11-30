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

Rectangle {
  id: stateAwareSpin
  height: stateAwareSpinContent.height
  color: "transparent"

  // step size
  property double stepValue: 0.1

  // min value
  property double minValue: 0

  // max value
  property double maxValue: 1

  // value of the spin number value
  property double numberValue: 0.0

  signal onChange(double _value)

  // Read-only / write
  property bool writeable: {
    var isModel = entityType == "model"
    return isModel || componentInspectorEditor.getSimPaused()
  }

  /**
   * Used to create a spin box
   */
  Component {
    id: writableNumber
    GzSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: minValue
      maximumValue: maxValue
      stepSize: stepValue
      decimals: {
        writableSpin.value = writableSpin.activeFocus ? writableSpin.value : numberValue
        stepSize == 1 ? 0 : componentInspectorEditor.getDecimals(writableSpin.width)
      }
      onEditingFinished: {
        numberValue = writableSpin.value
        onChange(writableSpin.value)
      }
    }
  }

  /**
   * Used to create a read-only number
   */
  Component {
    id: readOnlyNumber
    Rectangle {
      border.width: 1
      color: index % 2 == 0 ? lightGrey : darkGrey
      implicitHeight: 40
      Text {
        padding: 10
        id: numberText
        anchors.fill: parent
        horizontalAlignment: Text.AlignRight
        verticalAlignment: Text.AlignVCenter
        text: {
          return numberValue.toFixed(componentInspectorEditor.getDecimals(numberText.width))
        }
      }
    }
  }

  Column {
    anchors.fill: parent

    // Content
    Rectangle {
      id: stateAwareSpinContent
      width: parent.width
      height: stateAwareSpinContentLoader.height
      clip: true
      color: "transparent"

      Loader {
        id: stateAwareSpinContentLoader
        width: parent.width
        Layout.fillWidth: true
        sourceComponent: writeable ? writableNumber : readOnlyNumber
      }
    }
  }
}
