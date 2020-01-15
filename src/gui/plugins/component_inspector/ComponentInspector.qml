import QtQuick 2.9
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import IgnGazebo 1.0 as IgnGazebo

Rectangle {
  id: componentInspector
  color: "transparent"
  Layout.minimumWidth: 250
  Layout.minimumHeight: 375
  anchors.fill: parent

  /**
   * Time delay for tooltip to show, in ms
   */
  property int tooltipDelay: 500

  /**
   * Height of each item in pixels
   */
  property int itemHeight: 30

  /**
   * Light grey according to theme
   */
  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  /**
   * Dark grey according to theme
   */
  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  function delegateQml(_model) {
    if (_model === null || _model.dataType == undefined)
      return 'TypeHeader.qml'

    return _model.dataType + '.qml'
  }

  Label {
    id: entityLabel
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    text: 'Entity: ' + ComponentInspector.entity
    color: Material.theme == Material.Light ? "black" : "white"
    font.pointSize: 13
    padding: 3
  }

  ListView {
    anchors.top: entityLabel.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    model: ComponentInspectorModel
    spacing: 5

    delegate: Loader {
      id: loader
      source: delegateQml(model)
    }
  }
}
