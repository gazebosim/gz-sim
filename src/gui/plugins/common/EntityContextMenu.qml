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
      text: "Move To"
      onTriggered: context.OnRequest("move_to",
          context.entity);
    }
  }

  function open(_entity) {
    menu.x = mouse.mouseX
    menu.y = mouse.mouseY
    context.entity = _entity
    menu.open()
  }

  IgnGazebo.EntityContextMenuItem {
    id: context
    property string entity
  }
}


