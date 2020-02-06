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


