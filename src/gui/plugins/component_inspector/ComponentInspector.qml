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
import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.1
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import GzSim 1.0 as GzSim

Rectangle {
  id: componentInspector
  color: lightGrey
  Layout.minimumWidth: 400
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
   * Entity type
   */
  property string entityType: ComponentInspector.type

  /**
   * Get if entity is nested model or not
   */
  property bool nestedModel : ComponentInspector.nestedModel

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

  /**
   * Highlight color
   */
  property color highlightColor: Qt.rgba(
    Material.accent.r,
    Material.accent.g,
    Material.accent.b, 0.3)

  function delegateQml(_model) {
    if (_model === null || _model.dataType == undefined)
      return 'NoData.qml'

    return _model.dataType + '.qml'
  }

  // Get number of decimal digits based on a widget's width
  function getDecimals(_width) {
    if (_width <= 80)
      return 2;
    else if (_width <= 100)
      return 4;
    return 6;
  }

  /**
   * Forward pose changes to C++
   */
  function onPose(_x, _y, _z, _roll, _pitch, _yaw) {
    ComponentInspector.OnPose(_x, _y, _z, _roll, _pitch, _yaw)
  }

  /**
   * Forward light changes to C++
   */
  function onLight(_rSpecular, _gSpecular, _bSpecular, _aSpecular,
                   _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
                   _attRange, _attLinear, _attConstant, _attQuadratic,
                   _castShadows, _directionX, _directionY, _directionZ,
                   _innerAngle, _outerAngle, _falloff, _intensity, _type,
                   _isLightOn, _visualizeVisual) {
    ComponentInspector.OnLight(_rSpecular, _gSpecular, _bSpecular, _aSpecular,
                               _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
                               _attRange, _attLinear, _attConstant, _attQuadratic,
                               _castShadows, _directionX, _directionY, _directionZ,
                               _innerAngle, _outerAngle, _falloff, _intensity, _type,
                               _isLightOn, _visualizeVisual)
  }

  /*
   * Forward physics changes to C++
   */
  function onPhysics(_stepSize, _realTimeFactor) {
    ComponentInspector.OnPhysics(_stepSize, _realTimeFactor)
  }

  /**
   * Forward material color changes to C++
   */
  function onMaterialColor(_rAmbient, _gAmbient, _bAmbient, _aAmbient,
                           _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
                           _rSpecular, _gSpecular, _bSpecular, _aSpecular,
                           _rEmissive, _gEmissive, _bEmissive, _aEmissive,
                           _type, _currColor) {
    ComponentInspector.OnMaterialColor(
        _rAmbient, _gAmbient, _bAmbient, _aAmbient,
        _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
        _rSpecular, _gSpecular, _bSpecular, _aSpecular,
        _rEmissive, _gEmissive, _bEmissive, _aEmissive,
        _type, _currColor)
  }

  /*
   * Forward spherical coordinate changes to C++
   */
  function onSphericalCoordinates(_surface, _lat, _lon, _elevation, _heading) {
    ComponentInspector.OnSphericalCoordinates(_surface, _lat, _lon, _elevation,
        _heading);
  }

  Rectangle {
    id: header
    height: lockButton.height
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    width: parent.width
    color: darkGrey

    RowLayout {
      anchors.fill: parent
      spacing: 0

      GzSim.TypeIcon {
        id: icon
        height: lockButton.height * 0.8
        width: lockButton.height * 0.8
        entityType: ComponentInspector.type
      }

      Label {
        text: ComponentInspector.type
        font.capitalization: Font.Capitalize
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 3
      }

      Item {
        height: entityLabel.height
        Layout.fillWidth: true
      }

      ToolButton {
        id: lockButton
        checkable: true
        checked: false
        text: "Lock entity"
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: lockButton.checked ? "qrc:/Gazebo/images/lock.svg" :
                                       "qrc:/Gazebo/images/unlock.svg"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        ToolTip.text: lockButton.checked ? "Unlock entity" : "Lock entity"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onToggled: {
          ComponentInspector.locked = lockButton.checked
        }
      }

      ToolButton {
        id: pauseButton
        checkable: true
        checked: false
        text: pauseButton.checked ? "\u25B6" : "\u275A\u275A"
        contentItem: Text {
          text: pauseButton.text
          color: "#b5b5b5"
          horizontalAlignment: Text.AlignHCenter
          verticalAlignment: Text.AlignVCenter
        }
        ToolTip.text: pauseButton.checked ? "Resume updates" : "Pause updates"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onToggled: {
          ComponentInspector.paused = pauseButton.checked
        }
      }

      ToolButton {
        id: addSystemButton
        checkable: false
        text: "\u002B"
        contentItem: Text {
          text: addSystemButton.text
          color: "#b5b5b5"
          horizontalAlignment: Text.AlignHCenter
          verticalAlignment: Text.AlignVCenter
        }
        visible: (entityType == "model" || entityType == "visual" ||
                  entityType == "sensor" || entityType == "world")
        ToolTip.text: "Add a system to this entity"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onClicked: {
          addSystemDialog.open()
        }
      }


      Label {
        id: entityLabel
        text: 'Entity ' + ComponentInspector.entity
        Layout.minimumWidth: 80
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 5
      }
    }
  }

  Dialog {
    id: addSystemDialog
    modal: false
    focus: true
    title: "Add System"
    closePolicy: Popup.CloseOnEscape
    width: parent.width

    ColumnLayout {
      width: parent.width
      GridLayout {
        columns: 2
        columnSpacing: 30
        Text {
          text: "Name"
          Layout.row: 0
          Layout.column: 0
        }

        TextField {
          id: nameField
          selectByMouse: true
          Layout.row: 0
          Layout.column: 1
          Layout.fillWidth: true
          Layout.minimumWidth: 250
          onTextEdited: {
            addSystemDialog.updateButtonState();
          }
          placeholderText: "Leave empty to load first plugin"
        }

        Text {
          text: "Filename"
          Layout.row: 1
          Layout.column: 0
        }

        ComboBox {
          id: filenameCB
          Layout.row: 1
          Layout.column: 1
          Layout.fillWidth: true
          Layout.minimumWidth: 250
          model: ComponentInspector.systemNameList
          currentIndex: 0
          onCurrentIndexChanged: {
            if (currentIndex < 0)
              return;
            addSystemDialog.updateButtonState();
          }
          ToolTip.visible: hovered
          ToolTip.delay: tooltipDelay
          ToolTip.text: currentText
        }
      }

      Text {
        id: innerxmlLabel
        text: "Inner XML"
      }

      Flickable {
        id: innerxmlFlickable
        Layout.minimumHeight: 300
        Layout.fillWidth: true
        Layout.fillHeight: true

        flickableDirection: Flickable.VerticalFlick
        TextArea.flickable: TextArea {
          id: innerxmlField
          wrapMode: Text.WordWrap
          selectByMouse: true
          textFormat: TextEdit.PlainText
          font.pointSize: 10
        }
        ScrollBar.vertical: ScrollBar {
          policy: ScrollBar.AlwaysOn
        }
      }
    }

    footer: DialogButtonBox {
      id: buttons
      standardButtons: Dialog.Ok | Dialog.Cancel
    }

    onOpened: {
      ComponentInspector.QuerySystems();
      addSystemDialog.updateButtonState();
    }

    onAccepted: {
      ComponentInspector.OnAddSystem(nameField.text.trim(),
          filenameCB.currentText.trim(), innerxmlField.text.trim())
    }

    function updateButtonState() {
      buttons.standardButton(Dialog.Ok).enabled =
          (filenameCB.currentText.trim() != '')
    }
  }



  ListView {
    anchors.top: header.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    model: {
      try {
        return ComponentsModel;
      }
      catch (e) {
        // The QML is loaded before we set the context property
        return null
      }
    }
    spacing: 5

    delegate: Loader {
      id: loader

      /**
       * Most items can access model.data directly, but items like ListView,
       * which have their own `model` property, can't. They can use
       * `componentData` instead.
       */
      property var componentData: model.data

      source: delegateQml(model)
    }
  }
}
