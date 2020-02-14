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
import QtQuick 2.0
import QtQuick.Controls 2.0
import IgnGazebo 1.0 as IgnGazebo

Item {
  Menu {
    id: menu
    transformOrigin: Menu.TopRight
    MenuItem {
      id: moveToMenu
      text: "Move To"
      onTriggered: context.OnRequest("move_to", context.entity)
    }
    MenuItem {
      id: followMenu
      text: "Follow"
      onTriggered: context.OnRequest("follow", context.entity)
    }
    MenuItem {
      id: removeMenu
      text: "Remove"
      onTriggered: context.OnRemove(context.entity, context.type)
    }
  }

  function open(_entity, _type, _x, _y) {
    menu.x = _x
    menu.y = _y
    context.entity = _entity
    context.type = _type
    moveToMenu.enabled = false
    followMenu.enabled = false
    removeMenu.enabled = false

    // enable / disable menu items
    if (context.type == "model" || context.type == "link" ||
        context.type == "visual" || context.type == "light" ||
        context.type == "performer")
    {
      moveToMenu.enabled = true
      followMenu.enabled = true
    }

    if (context.type == "model" || context.type == "light")
    {
      removeMenu.enabled = true
    }

    menu.open()
  }

  IgnGazebo.EntityContextMenuItem {
    id: context
    property string entity
    property string type
  }
}


