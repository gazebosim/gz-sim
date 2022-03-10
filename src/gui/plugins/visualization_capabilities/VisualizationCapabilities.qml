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
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3

ColumnLayout {
  Layout.minimumWidth: 350
  Layout.minimumHeight: 130
  anchors.fill: parent
  anchors.margins: 10

  property string message: 'Adding visualization capabilities to the 3D ' +
      'scene such as view transparent, joints, CoM, etc.'

  Label {
    Layout.fillWidth: true
    wrapMode: Text.WordWrap
    text: message
  }

  Item {
    width: 10
    Layout.fillHeight: true
  }
}
