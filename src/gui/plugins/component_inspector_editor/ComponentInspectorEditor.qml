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
import QtQuick.Controls 1.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.0
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import GzSim 1.0 as GzSim


Rectangle {
  id: componentInspectorEditor
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
  property string entityType: ComponentInspectorEditor.type

  /**
   * Get if entity is nested model or not
   */
  property bool nestedModel : ComponentInspectorEditor.nestedModel

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

  /// \brief Get whether simulation is paused
  function getSimPaused() {
    return ComponentInspectorEditor.simPaused
  }

  // Get number of decimal digits based on a width value
  // \param[in] _width Pixel width
  // \return Number of decimals that fit with the provided width.
  function getDecimals(_width) {
    // Use full decimals if the width is <= 0, which allows the value
    // to appear correctly.
    if (_width <= 0 || _width > 110)
      return 6

    if (_width <= 80)
      return 2

    return 4
  }

  // Get number of decimal digits based on a widget's width, and adjust the
  // widget's value to prevent zero padding.
  // \param[in, out] _widgetId The widget id that will display the value. This
  // widget's width attribute is used to determine the number of decimals.
  // \param[in] _value The value that should be used to set the _widgetId's
  // value attribute.
  function getDecimalsAdjustValue(_widgetId, _value) {
    // Make sure to update the value, otherwise zeros are used intead of
    // the actual values.
    _widgetId.value = _widgetId.activeFocus ? _widgetId.value : _value
    return getDecimals(_widgetId.width)
  }

  /**
   * Forward light changes to C++
   */
  function onLight(_rSpecular, _gSpecular, _bSpecular, _aSpecular,
                   _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
                   _attRange, _attLinear, _attConstant, _attQuadratic,
                   _castShadows, _directionX, _directionY, _directionZ,
                   _innerAngle, _outerAngle, _falloff, _intensity, _type) {
    ComponentInspectorEditor.OnLight(_rSpecular, _gSpecular, _bSpecular, _aSpecular,
                               _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
                               _attRange, _attLinear, _attConstant, _attQuadratic,
                               _castShadows, _directionX, _directionY, _directionZ,
                               _innerAngle, _outerAngle, _falloff, _intensity, _type)
  }

  /*
   * Forward physics changes to C++
   */
  function onPhysics(_stepSize, _realTimeFactor) {
    ComponentInspectorEditor.OnPhysics(_stepSize, _realTimeFactor)
  }

  /**
   * Forward material color changes to C++
   */
  function onMaterialColor(_rAmbient, _gAmbient, _bAmbient, _aAmbient,
                           _rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse,
                           _rSpecular, _gSpecular, _bSpecular, _aSpecular,
                           _rEmissive, _gEmissive, _bEmissive, _aEmissive,
                           _type, _currColor) {
    ComponentInspectorEditor.OnMaterialColor(
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
    ComponentInspectorEditor.OnSphericalCoordinates(_surface, _lat, _lon, _elevation,
        _heading);
  }

  // The component for a menu section header
  Component {
    id: menuSectionHeading
    Rectangle {
      height: childrenRect.height

      Text {
          text: sectionText
          font.pointSize: 10
          padding: 5
      }
    }
  }

  Dialog {
    id: jointDialog
    modal: true
    focus: true
    header: ColumnLayout {
      id: jointAddHeader
      Text {
        text:"Add joint"
        font.pointSize: 14
        padding: 20
      }

      Text {
        text:"Select the parent and child links"
        font.pointSize: 12
        leftPadding: 20
        rightPadding: 20
        bottomPadding: 20
      }
    }

    standardButtons: Dialog.Ok | Dialog.Cancel

    property string jointType: "empty"

    contentItem: ColumnLayout {
      Text {
        id: parentBoxText
        text: "Parent Link"
      }
      ComboBox {
        id: parentBox
        model: ComponentInspectorEditor.modelParentLinks
        currentIndex: 0
      }
      Text {
        id: childBoxText
        text: "Child Link"
      }
      ComboBox {
        id: childBox
        model: ComponentInspectorEditor.modelChildLinks
        currentIndex: 1
      }
    }

    onAccepted: {
      ComponentInspectorEditor.OnAddJoint(jointType, parentBox.currentText, childBox.currentText)
    }
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
        entityType: ComponentInspectorEditor.type
      }

      Label {
        text: ComponentInspectorEditor.type
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
          ComponentInspectorEditor.locked = lockButton.checked
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
          ComponentInspectorEditor.paused = pauseButton.checked
        }
      }

      ToolButton {
        id: addLinkButton
        checkable: false
        text: "Add entity"
        visible: entityType == "model"
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: "qrc:/Gazebo/images/plus-link.png"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        ToolTip.text: "Add a link or light to a model"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onClicked: {
          getSimPaused() ? addLinkMenu.open() : linkAddPausePopup.open()
        }
        Popup {
          id: linkAddPausePopup
          modal: true
          focus: true
          x: parent.width - linkAdPopupContentText.width
          y: parent.height + linkAdPopupContentText.height
          contentItem: Text {
            id: linkAdPopupContentText
            padding: 10
            text: "Pause simulation to add a link, light, or joint"
          }
          closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent
        }

        FileDialog {
          id: loadFileDialog
          title: "Load mesh"
          folder: shortcuts.home
          nameFilters: [ "Collada files (*.dae)", "(*.stl)", "(*.obj)" ]
          selectMultiple: false
          selectExisting: true
          onAccepted: {
            ComponentInspectorEditor.OnLoadMesh("mesh", "link", fileUrl)
          }
        }

        Menu {
          id: addLinkMenu

          Item {
            Layout.fillWidth: true
            height: childrenRect.height
            Loader {
              property string sectionText: "Link"
              sourceComponent: menuSectionHeading
            }
          }

          MenuItem {
            id: boxLink
            text: "Box"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("box", "link");
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: capsuleLink
            text: "Capsule"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("capsule", "link");
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: coneLink
            text: "Cone"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("cone", "link");
            }
          }

          MenuItem {
            id: cylinderLink
            text: "Cylinder"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("cylinder", "link");
            }
          }

          MenuItem {
            id: ellipsoidLink
            text: "Ellipsoid"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("ellipsoid", "link");
            }
          }

          MenuItem {
            id: emptyLink
            text: "Empty"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("empty", "link");
            }
          }

          MenuItem {
            id: meshLink
            text: "Mesh"
            onClicked: {
              loadFileDialog.open()
            }
          }

          MenuItem {
            id: sphereLink
            text: "Sphere"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("sphere", "link");
            }
          }

          MenuSeparator {
            padding: 0
            topPadding: 12
            bottomPadding: 12
            contentItem: Rectangle {
              implicitWidth: 200
              implicitHeight: 1
              color: "#1E000000"
            }
          }

          Item {
            Layout.fillWidth: true
            height: childrenRect.height
            Loader {
              property string sectionText: "Light"
              sourceComponent: menuSectionHeading
            }
          }

          MenuItem {
            id: directionalLink
            text: "Directional"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("directional", "link");
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: pointLink
            text: "Point"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("point", "link");
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: spotLink
            text: "Spot"
            onClicked: {
              ComponentInspectorEditor.OnAddEntity("spot", "link");
              addLinkMenu.close()
            }
          }

          MenuSeparator {
            padding: 0
            topPadding: 12
            bottomPadding: 12
            contentItem: Rectangle {
              implicitWidth: 200
              implicitHeight: 1
              color: "#1E000000"
            }
          }

          Item {
            Layout.fillWidth: true
            height: childrenRect.height
            Loader {
              property string sectionText: "Joint"
              sourceComponent: menuSectionHeading
            }
          }

          MenuItem {
            id: ballJoint
            text: "Ball"
            onClicked: {
              jointDialog.jointType = "ball"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: continuousJoint
            text: "Continuous"
            onClicked: {
              jointDialog.jointType = "continuous"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: fixedJoint
            text: "Fixed"
            onClicked: {
              jointDialog.jointType = "fixed"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: gearboxJoint
            text: "Gearbox"
            onClicked: {
              jointDialog.jointType = "gearbox"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: prismaticJoint
            text: "Prismatic"
            onClicked: {
              jointDialog.jointType = "prismatic"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: revoluteJoint
            text: "Revolute"
            onClicked: {
              jointDialog.jointType = "revolute"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: revolute2Joint
            text: "Revolute2"
            onClicked: {
              jointDialog.jointType = "revolute2"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: screwJoint
            text: "Screw"
            onClicked: {
              jointDialog.jointType = "screw"
              jointDialog.open()
              addLinkMenu.close()
            }
          }

          MenuItem {
            id: universalJoint
            text: "Universal"
            onClicked: {
              jointDialog.jointType = "universal"
              jointDialog.open()
              addLinkMenu.close()
            }
          }
        }
      }

      ToolButton {
        id: addSensorButton
        checkable: false
        text: "Add sensor"
        visible: entityType == "link"
        contentItem: Image {
          fillMode: Image.Pad
          horizontalAlignment: Image.AlignHCenter
          verticalAlignment: Image.AlignVCenter
          source: "qrc:/Gazebo/images/plus-sensor.png"
          sourceSize.width: 18;
          sourceSize.height: 18;
        }
        ToolTip.text: "Add a sensor to a link"
        ToolTip.visible: hovered
        ToolTip.delay: Qt.styleHints.mousePressAndHoldInterval
        onClicked: {
          getSimPaused() ? addSensorMenu.open() : sensorAddPausePopup.open()
        }
        Popup {
          id: sensorAddPausePopup
          modal: true
          focus: true
          x: parent.width - sensorAddPopupContentText.width
          y: parent.height + sensorAddPopupContentText.height
          contentItem: Text {
            id: sensorAddPopupContentText
            padding: 10
            text: "Pause simulation to add a sensor"
          }
          closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutsideParent
        }

        Menu {
          id: addSensorMenu
          MenuItem {
            id: airPressure
            text: "Air pressure"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(airPressure.text, "sensor");
            }
          }

          MenuItem {
            id: altimeter
            text: "Altimeter"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(altimeter.text, "sensor");
            }
          }

          MenuItem {
            id: cameraSensorMenu
            text: "Camera >"

            MouseArea {
              id: viewSubCameraArea
              anchors.fill: parent
              hoverEnabled: true
              onEntered: cameraSubmenu.open()
            }
          }

          MenuItem {
            id: contact
            text: "Contact"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(contact.text, "sensor");
            }
          }

          MenuItem {
            id: forceTorque
            text: "Force torque"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(forceTorque.text, "sensor");
            }
          }

          /*MenuItem {
            id: gps
            text: "GPS"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(gps.text, "sensor");
            }
          }*/

          MenuItem {
            id: gpuLidar
            text: "GPU Lidar"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("gpu_lidar", "sensor");
            }
          }

          MenuItem {
            id: imu
            text: "IMU"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(imu.text, "sensor");
            }
          }

          MenuItem {
            id: magnetometer
            text: "Magnetometer"
            onTriggered: {
              ComponentInspectorEditor.OnAddEntity(magnetometer.text, "sensor");
            }
          }
        }

        Menu {
          id: cameraSubmenu
          x: addSensorMenu.x - addSensorMenu.width
          y: addSensorMenu.y + cameraSensorMenu.y

          MenuItem {
            id: depth
            text: "Depth"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("depth_camera", "sensor");
            }
          }
          MenuItem {
            id: logical
            text: "Logical"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("logical_camera", "sensor");
            }
          }
          MenuItem {
            id: monocular
            text: "Monocular"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("camera", "sensor");
            }
          }
          /*MenuItem {
            id: multicamera
            text: "Multicamera"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("multicamera", "sensor");
            }
          }*/
          MenuItem {
            id: rgbd
            text: "RGBD"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("rgbd_camera", "sensor");
            }
          }
          MenuItem {
            id: segmentation
            text: "Segmentation"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("segmentation_camera", "sensor");
            }
          }
          MenuItem {
            id: thermal
            text: "Thermal"
             onTriggered: {
              ComponentInspectorEditor.OnAddEntity("thermal_camera", "sensor");
            }
          }
        }
      }

      Label {
        id: entityLabel
        text: 'Entity ' + ComponentInspectorEditor.entity
        Layout.minimumWidth: 80
        color: Material.theme == Material.Light ? "#444444" : "#cccccc"
        font.pointSize: 12
        padding: 5
      }
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
      source: delegateQml(model)
    }
  }
}
