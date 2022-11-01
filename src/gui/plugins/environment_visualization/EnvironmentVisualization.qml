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
import QtQuick.Controls 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import "qrc:/qml"


GridLayout {
  columns: 8
  columnSpacing: 10
  Layout.minimumWidth: 350
  Layout.minimumHeight: 400
  anchors.fill: parent
  anchors.leftMargin: 10
  anchors.rightMargin: 10

  Label {
    Layout.row: 0
    Layout.columnSpan: 8
    horizontalAlignment: Text.AlignCenter
    id: instructionLabel
    color: "dimgrey"
    text: qsTr("For the actual pointcloud please open the point_cloud panel")
  }

  Label {
    Layout.row: 1
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: dimensionLabelX
    color: "dimgrey"
    text: qsTr("X Samples")
  }

  Slider {
    Layout.row: 1
    Layout.column: 2
    Layout.columnSpan: 6
    id: stepSliderX
    from: 5
    value: 10
    to: 50
    onMoved: function() {
      EnvironmentVisualization.xSamples = value;
      EnvironmentVisualization.ResamplePointcloud();
    }
  }

  Label {
    Layout.row: 2
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: dimensionLabelY
    color: "dimgrey"
    text: qsTr("Y Samples")
  }

  Slider {
    Layout.row: 2
    Layout.column: 2
    Layout.columnSpan: 6
    id: stepSliderY
    from: 5
    value: 10
    to: 50
    onMoved: function() {
      EnvironmentVisualization.ySamples = value;
      EnvironmentVisualization.ResamplePointcloud();
    }
  }

  Label {
    Layout.row: 3
    Layout.columnSpan: 2
    horizontalAlignment: Text.AlignRight
    id: dimensionLabelZ
    color: "dimgrey"
    text: qsTr("Z Samples")
  }

  Slider {
    Layout.row: 3
    Layout.column: 2
    Layout.columnSpan: 6
    id: stepSliderZ
    from: 5
    value: 10
    to: 50
    onMoved: function() {
      EnvironmentVisualization.zSamples = value;
      EnvironmentVisualization.ResamplePointcloud();
    }
  }
}
