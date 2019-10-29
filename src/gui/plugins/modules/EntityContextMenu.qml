import QtQuick 2.0
import QtQuick.Controls 2.0
import IgnGazebo 1.0 as IgnGazebo

Item {
  id: content

  MouseArea {
    id: mouse
    anchors.fill: content
    hoverEnabled: true
    acceptedButtons: Qt.NoButton
  }

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
  }

  function open(_entity, _type) {
    menu.x = mouse.mouseX
    menu.y = mouse.mouseY
    context.entity = _entity
    context.type = _type
    moveToMenu.enabled = false
    followMenu.enabled = false

    // enable / disable menu items
    if (context.type == "model" || context.type == "link" ||
        context.type == "visual" || context.type == "light" ||
        context.type == "performer")
    {
      moveToMenu.enabled = true
      followMenu.enabled = true
    }

    menu.open()
  }

  IgnGazebo.EntityContextMenuItem {
    id: context
    property string entity
    property string type
  }
}


