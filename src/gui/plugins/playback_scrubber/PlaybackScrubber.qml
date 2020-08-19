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
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4

ToolBar {
  id: shapes
  Layout.minimumWidth: 200
  Layout.minimumHeight: 100
  property var isPressed: false

  function updateSliderValue() {
    if (!shapes.isPressed)
    {
      print("the progress is ");
      print(PlaybackScrubber.Progress());
      slider.value = PlaybackScrubber.Progress();
    }
  }

  background: Rectangle {
    color: "transparent"
  }

  Connections {
    target: PlaybackScrubber
    onNewProgress: updateSliderValue()
  }

  Slider {
    id: slider
    from: 0
    value: updateSliderValue()
    to: 1
    stepSize: 0.001
    Layout.alignment: Qt.AlignVCenter
    /*
    onValueChanged: {
      print(slider.value)
    }
    */
    onPressedChanged: {
      if (!pressed)
      {
        PlaybackScrubber.OnDrag(slider.value, slider.from, slider.to);
        shapes.isPressed = false;
      }
      else
      {
        shapes.isPressed = true;
      }
    }
  }
}
