/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
import IgnGazebo 1.0 as IgnGazebo

Rectangle {
  id: componentInspector
  color: lightGrey
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375
  anchors.fill: parent

  /**
   * Time delay for tooltip to show, in ms
   */
  property int tooltipDelay: 500

  /**
   * Height of each item in pixels
   */
  property int itemHeight: 30

  /**
   * Light grey according to theme
   */
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Dark grey according to theme
   */
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  function delegateQml(_model) {
    if (_model === null || _model.dataType == undefined)
      return 'TypeHeader.qml'

    return _model.dataType + '.qml'
  }

  Rectangle {
    id: header
    height: entityLabel.height
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    width: parent.width
    color: darkGrey


    IgnGazebo.TypeIcon {
      id: icon
      anchors.left: parent.left
      height: entityLabel.height
      width: entityLabel.height
      entityType: ComponentInspector.type
    }

    Label {
      anchors.left: parent.left
      text: ComponentInspector.type
      font.capitalization: Font.Capitalize
      color: Material.theme == Material.Light ? "#444444" : "#cccccc"
      font.pointSize: 14
      padding: 3
      leftPadding: entityLabel.height
    }

    Label {
      id: entityLabel
      anchors.right: parent.right
      text: 'Entity ' + ComponentInspector.entity
      color: Material.theme == Material.Light ? "#444444" : "#cccccc"
      font.pointSize: 14
      padding: 3
    }
  }

  ListView {
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    model: ComponentInspectorModel
    spacing: 5

    delegate: Loader {
      id: loader
      source: delegateQml(model)
    }
  }
}
