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
import QtQuick 2.0
import QtQuick.Controls 2.0

Image {
  id: typeIcon
  sourceSize.height: typeIcon.height
  sourceSize.width: typeIcon.width
  fillMode: Image.PreserveAspectFit
  horizontalAlignment: Image.AlignHCenter
  verticalAlignment: Image.AlignLeft
  source: image()

  /**
   * Entity type, such as 'world' or 'model'.
   */
  property string entityType: ''

  property int tooltipDelay: 500

  /**
   * Image address according to type
   */
  function image() {
    // Use box icon by default
    var entityImage = 'visual'

    // Other specific icons available
    if (entityType == 'actor' ||
        entityType == 'collision' ||
        entityType == 'joint' ||
        entityType == 'light' ||
        entityType == 'link' ||
        entityType == 'sensor' ||
        entityType == 'model')
    {
      entityImage = entityType
    }

    return 'qrc:/Gazebo/images/' + entityImage + '.png'
  }

  ToolTip {
    visible: iconMa.containsMouse
    delay: tooltipDelay
    text: entityType
    y: icon.z - 30
    enter: null
    exit: null
  }

  MouseArea {
    id: iconMa
    anchors.fill: parent
    hoverEnabled: true
  }
}
