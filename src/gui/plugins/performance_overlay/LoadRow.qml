/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

// Sibling components. Qt 5 does not find them in qrc without this.
import "qrc:/PerformanceOverlay"

// A ranked row: a name, a bar split by kind of load, and the share of the time
Item {
  id: row

  property string name: ""
  // Prefix of the value, for what isn't measured directly
  property string prefix: ""
  property string tooltip: name
  property real value: 0
  // Value that fills the whole bar
  property real whole: 1
  property var parts: []
  property bool clickable: false
  property color trackColor: "transparent"

  signal clicked()

  implicitHeight: 22
  Accessible.role: clickable ? Accessible.Button : Accessible.StaticText
  Accessible.name: name + " " + valueLabel.text

  Rectangle {
    anchors.fill: parent
    radius: 4
    color: hover.containsMouse && row.clickable ? row.trackColor : "transparent"
  }

  RowLayout {
    anchors.fill: parent
    anchors.leftMargin: 4
    anchors.rightMargin: 4
    spacing: 8

    Label {
      text: row.name
      elide: Text.ElideMiddle
      font.pixelSize: 12
      Layout.preferredWidth: 130
    }

    StackedBar {
      parts: row.parts
      whole: row.whole
      trackColor: row.trackColor
      Layout.fillWidth: true
      Layout.preferredHeight: 6
      Layout.alignment: Qt.AlignVCenter
    }

    Label {
      id: valueLabel
      text: row.prefix + Math.round(100 * row.value) + "%"
      horizontalAlignment: Text.AlignRight
      font.pixelSize: 12
      Layout.preferredWidth: 42
    }
  }

  MouseArea {
    id: hover
    anchors.fill: parent
    hoverEnabled: true
    cursorShape: row.clickable ? Qt.PointingHandCursor : Qt.ArrowCursor
    ToolTip.visible: hover.containsMouse
    ToolTip.delay: 300
    ToolTip.text: row.tooltip
    onClicked: {
      if (row.clickable)
        row.clicked()
    }
  }
}
