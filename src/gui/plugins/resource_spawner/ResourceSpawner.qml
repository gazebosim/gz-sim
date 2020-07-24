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
import QtQuick.Controls 1.4
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import "qrc:/ResourceSpawner"

import QtQml.Models 2.2
ColumnLayout {
  id: resourceSpawner
  Layout.minimumWidth: 400
  Layout.minimumHeight: 700
  anchors.fill: parent
  spacing: 2

  /**
   * Height of each item on the tree
   */
  property int treeItemHeight: 30;

  /**
   * Height of each item on the grid
   */
  property int gridItemHeight: 150;

  /**
   * Width of each item on the grid
   */
  property int gridItemWidth: 200;

  /**
   * Currently selected path
   */
  property string currentPath: "";

  /**
   * Currently selected owner
   */
  property string currentOwner: "";

  /**
   * Color for odd rows, according to theme
   */
  property color oddColor: (Material.theme == Material.Light) ?
  Material.color(Material.Grey, Material.Shade100):
  Material.color(Material.Grey, Material.Shade800);

  /**
   * Color for even rows, according to theme
   */
  property color evenColor: (Material.theme == Material.Light) ?
  Material.color(Material.Grey, Material.Shade200):
  Material.color(Material.Grey, Material.Shade900);
  Rectangle {
    id: rect1
    width: resourceSpawner.width
    height: 35
    color: "transparent"
    border.color: "black"
    Text {
      text: "Local Models"
      font.pointSize: 15
      leftPadding: 5
      topPadding: 5
    }
  }

  ResourceView {
    id: resourceView
  }

  Rectangle {
    id: rect2
    width: resourceSpawner.width
    height: 35
    color: "transparent"
    border.color: "black"
    Text {
      text: "Fuel Models"
      font.pointSize: 15
      leftPadding: 5
      topPadding: 5
    }
  }

  FuelView {
    id: fuelView
  }
}
