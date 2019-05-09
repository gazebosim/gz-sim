import QtQuick 2.0
import QtQuick.Controls 2.0
import RenderWindow 1.0
import QtGraphicalEffects 1.0

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
}


