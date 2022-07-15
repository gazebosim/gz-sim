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
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import "qrc:/JointPositionController"
import "qrc:/qml"

Rectangle {
  id: joint
  height: slider.height
  width: jointPositionController.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Position target value
  property double targetValue: 0.0

  // Horizontal margins
  property int margin: 15

  Connections {
    target: joint
    onTargetValueChanged: {
      jointPositionController.onCommand(model.name, joint.targetValue);
    }
  }

  RowLayout {
    anchors.fill: parent
    spacing: 0

    Item {
      height: parent.height
      width: margin
    }

    Text {
      text: model.name
      Layout.alignment: Qt.AlignVCenter
      Layout.preferredWidth: 100
      elide: Text.ElideRight
      ToolTip {
        visible: ma.containsMouse
        delay: Qt.styleHints.mousePressAndHoldInterval
        text: model.name
        y: -30
        enter: null
        exit: null
      }
      MouseArea {
        id: ma
        anchors.fill: parent
        hoverEnabled: true
        acceptedButtons: Qt.RightButton
      }
    }

    GzSpinBox {
      id: spin
      value: spin.activeFocus ? joint.targetValue : model.value
      minimumValue: model.min
      maximumValue: model.max
      decimals: 2
      stepSize: 0.1
      onEditingFinished: {
        joint.targetValue = spin.value
      }
    }

    Text {
      text: model.min.toFixed(2)
      Layout.alignment: Qt.AlignVCenter
    }

    Slider {
      id: slider
      Layout.alignment: Qt.AlignVCenter
      Layout.fillWidth: true
      from: model.min
      to: model.max
      value: slider.activeFocus ? joint.targetValue : model.value
      onMoved: {
        joint.targetValue = slider.value
      }
    }

    Text {
      text: model.max.toFixed(2)
      Layout.alignment: Qt.AlignVCenter
    }

    Item {
      height: parent.height
      width: margin
    }
  }
}
