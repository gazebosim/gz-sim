import QtQuick 2.9
import QtQuick.Controls 2.0
import RenderWindow 1.0
import QtGraphicalEffects 1.0
import IgnGazebo 1.0 as IgnGazebo

Rectangle {
  width: 1000
  height: 800

  /**
   * True to enable gamma correction
   */
  property bool gammaCorrect: false

  RenderWindow {
    id: renderWindow
    objectName: "renderWindow"
    anchors.fill: parent

    /**
     * Message to be displayed over the render window
     */
    property string message: ""

    Connections {
      target: renderWindow
      onOpenContextMenu: entityContextMenu.open(_entity, "model");
    }
  }

  /*
   * Gamma correction for sRGB output. Enabled when engine is set to ogre2
   */
  GammaAdjust {
      anchors.fill: renderWindow
      source: renderWindow
      gamma: 2.4
      enabled: gammaCorrect
      visible: gammaCorrect
  }

  onParentChanged: {
    if (undefined === parent)
      return;

      width = Qt.binding(function() {return parent.parent.width})
      height = Qt.binding(function() {return parent.parent.height})
  }

  IgnGazebo.EntityContextMenu {
    id: entityContextMenu
    anchors.fill: parent
  }

  // todo(anyone) replace this with snackbar notifications
  Text {
    text: renderWindow.message
  }

  DropArea {
    anchors.fill: renderWindow

    onDropped: {
      GzScene3D.OnDropped(drop.text)
    }
  }
}
