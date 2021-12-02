/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
import "qrc:/ComponentInspector"
import "qrc:/qml"

// Item displaying camera sensor information.
Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Image width
  property double imageWidth: model.data[0]

  // Image height
  property double imageHeight: model.data[1]

  // Horizontal field of view 
  property double horizontalFov: model.data[2]

  // Near clip
  property double nearClip: model.data[3]

  // Far clip
  property double farClip: model.data[4]

  // Image noise mean
  property double imageNoiseMean: model.data[5]

  // Image noise std dev
  property double imageNoiseStdDev: model.data[6]

  // Callback when image width is changed
  function onImageWidth(_value) {
    imageWidth = _value
    CameraImpl.OnImageSizeChange(imageWidth, imageHeight)
  }

  // Callback when image height is changed
  function onImageHeight(_value) {
    imageHeight = _value
    CameraImpl.OnImageSizeChange(imageWidth, imageHeight)
  }

  // Callback when near clip is changed
  function onNearClip(_value) {
    nearClip = _value
    CameraImpl.OnClipChange(nearClip, farClip)
  }

  // Callback when image height is changed
  function onFarClip(_value) {
    farClip = _value
    CameraImpl.OnClipChange(nearClip, farClip)
  }

  // Callback when horizontal field of view is changed
  function onHFov(_value) {
    horizontalFov = _value
    CameraImpl.OnHorizontalFovChange(horizontalFov)
  }

  Column {
    anchors.fill: parent

    // The expanding header. Make sure that the content to expand has an id set
    // to the value "content".
    ExpandingTypeHeader {
      id: header

      // Set the 'expandingHeaderText' value to override the default header
      // values, which is based on the model.
      expandingHeaderText: "Camera"
      expandingHeaderToolTip: "Camera sensor properties"
    }

    // This is the content that will be expanded/contracted using the
    // ExpandingHeader above. Note the "id: content" attribute.
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? layout.height : 0
      clip: true
      color: "transparent"
      Layout.leftMargin: 4

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      ColumnLayout {
        id: layout
        width: parent.width

        // Horizontal field of view section header.
        Rectangle {
          width: parent.width
          Layout.topMargin: 10
          color: "transparent"
        }
     
        // Horizontal field of view 
        RowLayout {
          id: hfovRowLayout
          width: layout.width

          Text {
            id : hfovText
            text: 'Horizontal field of view (r)'
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            leftPadding: 10
            ToolTip {
              visible: hfovMa.containsMouse
              delay: tooltipDelay
              text: "Horizontal field of view in radians"
              y: hfovText.y + 30
              enter: null
              exit: null
            }
            MouseArea {
              id: hfovMa
              anchors.fill: parent
              hoverEnabled: true
              acceptedButtons: Qt.RightButton
            }
          }
          StateAwareSpin {
            id: hfovSpin
            Layout.fillWidth: true
            height: 40
            numberValue: horizontalFov
            minValue: 0.1
            maxValue: 6.283186
            stepValue: 0.1
            // Connect to the onChange signal
            Component.onCompleted: {
              hfovSpin.onChange.connect(onHFov)
            }
          }
        }
        // End of horizontal field of view 
 
        // Image size header.
        Rectangle {
          id: imageSizeTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Image size"
            color: "dimgrey"
            font.pointSize: 12
          }
        }
        // Image size properties
        Rectangle {
          id: imageSizeRect
          width: parent.width
          Layout.fillWidth: true
          height: imageSizeGrid.height
          color: "transparent"
          Layout.leftMargin: 20
    
          GridLayout {
            id: imageSizeGrid
            width: parent.width
            columns: 4
    
            // Padding 
            Rectangle {
              Layout.columnSpan: 4
              height: 4
            }
    
            // Image width
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: imageWidthText.width
    
              Text {
                id : imageWidthText
                text: 'Width (px)'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: imageWidthMa.containsMouse
                  delay: tooltipDelay
                  text: "Image width in pixels"
                  y: imageWidthText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: imageWidthMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: imageWidthSpin
              Layout.fillWidth: true
              height: 40
              numberValue: imageWidth
              minValue: 1
              maxValue: 100000
              stepValue: 1
              // Connect to the onChange signal
              Component.onCompleted: {
                imageWidthSpin.onChange.connect(onImageWidth)
              }
            }
            // End of image width
            
            // Image height
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: imageHeightText.width
    
              Text {
                id : imageHeightText
                text: 'Height (px)'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: imageHeightMa.containsMouse
                  delay: tooltipDelay
                  text: "Image height in pixels"
                  y: imageHeightText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: imageHeightMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: imageHeightSpin
              Layout.fillWidth: true
              height: 40
              numberValue: imageHeight
              minValue: 1
              maxValue: 100000
              stepValue: 1
              // Connect to the onChange signal
              Component.onCompleted: {
                imageHeightSpin.onChange.connect(onImageHeight)
              }
            }
            // End of image width
          }
          // End of image size grid layout 
        }
        // End of image size rectangle
        
        
        // Clip distance header.
        Rectangle {
          id: clipDistTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            text: "Clip range"
            color: "dimgrey"
            font.pointSize: 12
          }
        }
        // Clip range properties
        Rectangle {
          id: clipDistRect
          width: parent.width
          Layout.fillWidth: true
          height: imageSizeGrid.height
          color: "transparent"
          Layout.leftMargin: 20
    
          GridLayout {
            id: clipDistGrid
            width: parent.width
            columns: 4
    
            // Padding 
            Rectangle {
              Layout.columnSpan: 4
              height: 4
            }
    
            // Near clip
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: nearClipText.width
    
              Text {
                id : nearClipText
                text: 'Near (m)'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: nearClipMa.containsMouse
                  delay: tooltipDelay
                  text: "Near clip distance in meters"
                  y: nearClipText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: nearClipMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: nearClipSpin
              Layout.fillWidth: true
              height: 40
              numberValue: nearClip
              minValue: 0
              maxValue: 100000
              stepValue: 0.1
              // Connect to the onChange signal
              Component.onCompleted: {
                nearClipSpin.onChange.connect(onNearClip)
              }
            }
            // End of near clip
            
            // Far clip
            Rectangle {
              color: "transparent"
              height: 40
              Layout.preferredWidth: farClipText.width
    
              Text {
                id : farClipText
                text: 'Far (m)'
                color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
                font.pointSize: 12
                anchors.centerIn: parent
                ToolTip {
                  visible: farClipMa.containsMouse
                  delay: tooltipDelay
                  text: "Far clip distance in meters"
                  y: farClipText.y + 30
                  enter: null
                  exit: null
                }
                MouseArea {
                  id: farClipMa
                  anchors.fill: parent
                  hoverEnabled: true
                  acceptedButtons: Qt.RightButton
                }
              }
            }
            StateAwareSpin {
              id: farClipSpin
              Layout.fillWidth: true
              height: 40
              numberValue: farClip
              minValue: 0.1
              maxValue: 100000
              stepValue: 0.1
              // Connect to the onChange signal
              Component.onCompleted: {
                farClipSpin.onChange.connect(onFarClip)
              }
            }
            // End of far clip 
          }
          // End of clip distance grid layout 
        }
        // End of clip distance rectangle
 
       
        // Image noise section header.
        /* \todo (nkoenig) Enable this in Ignition Garden
        Rectangle {
          id: verticalPositionNoiseTextRect
          width: parent.width
          height: childrenRect.height
          Layout.leftMargin: 10
          Layout.topMargin: 10
          color: "transparent"

          Text {
            id: verticalPositionNoiseText
            text: "Image noise"
            color: "dimgrey"
            font.pointSize: 12
          }
        }

        // Show the position noise values.
        NoiseSimple {
          id: imageNoise
          Layout.fillWidth: true
          Layout.leftMargin: 20

          meanValue: imageNoiseMean
          stdDevValue: imageNoiseStdDev 

          // Connect to the onNoiseUpdate signal in NoiseSimple.qml
          Component.onCompleted: {
            imageNoise.onNoiseUpdate.connect(
                CameraImpl.OnImageNoise)
          }
        }*/
      }
    }
  }
}
