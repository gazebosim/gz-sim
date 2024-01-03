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

// Item displaying physics information.
Rectangle {
  height: header.height + content.height
  width: componentInspector.width
  color: index % 2 == 0 ? lightGrey : darkGrey

  // Left indentation
  property int indentation: 10

  // Minimum parameter value, step size and RTF must be strictly positive
  property double minPhysParam: 0.000001

  // Maximum parameter value
  property double maxPhysParam: 100000

  // Horizontal margins
  property int margin: 5

  property int iconWidth: 20
  property int iconHeight: 20

  // Loaded item for physics step size
  property var stepSizeItem: {}

  // Loaded item for real time factor
  property var realTimeFactorItem: {}

  // Send new physics data to C++
  function sendPhysics() {
    // TODO(anyone) There's a loss of precision when these values get to C++
    componentInspector.onPhysics(
      stepSizeItem.value,
      realTimeFactorItem.value
    );
  }

  FontMetrics {
    id: fontMetrics
    font.family: "Roboto"
  }

  /**
   * Used to create a spin box
   */

  Component {
    id: writablePositiveNumber
    GzSpinBox {
      id: writableSpin
      value: writableSpin.activeFocus ? writableSpin.value : numberValue
      minimumValue: minPhysParam
      maximumValue: maxPhysParam
      decimals: 6
      stepSize: 0.001
      onEditingFinished: {
        sendPhysics()
      }
    }
  }

  Component {
    id: gzPlotIcon
    GzPlotIcon {
      gzMimeData: { "text/plain" : (model === null) ? "" :
        "Component," + model.entity + "," + model.typeId + "," +
        model.dataType + "," + gzComponentInfo + "," + model.shortName
      }
    }
  }

  Column {
    anchors.fill: parent

    // The expanding header. Make sure that the content to expand has an id set
    // to the value "content".
    ExpandingTypeHeader {
      id: header
      // Using the default header text values.
    }

    // Content
    Rectangle {
      id: content
      property bool show: false
      width: parent.width
      height: show ? grid.height : 0
      clip: true
      color: "transparent"

      Behavior on height {
        NumberAnimation {
          duration: 200;
          easing.type: Easing.InOutQuad
        }
      }

      GridLayout {
        id: grid
        width: parent.width
        columns: 3

        // Left spacer
        Item {
          Layout.rowSpan: 3
          width: margin + indentation
        }

        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: stepSizeText.width + indentation*3
          Loader {
            id: loaderStepSize
            sourceComponent: gzPlotIcon
            property string gzComponentInfo: "stepSize"
          }

          Text {
            id : stepSizeText
            text: ' Step Size (s)'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: stepSizeLoader
            anchors.fill: parent
            property double numberValue: model.data[0]
            sourceComponent: writablePositiveNumber
            onLoaded: {
              stepSizeItem = stepSizeLoader.item
            }
          }
        }
        Rectangle {
          color: "transparent"
          height: 40
          Layout.preferredWidth: realTimeFactorText.width + indentation*3
          Loader {
            id: loaderRealTimeFactor
            sourceComponent: gzPlotIcon
            property string gzComponentInfo: "realTimeFactor"
          }

          Text {
            id : realTimeFactorText
            text: ' Real time factor'
            leftPadding: 5
            color: Material.theme == Material.Light ? "#444444" : "#bbbbbb"
            font.pointSize: 12
            anchors.centerIn: parent
          }
        }
        Item {
          Layout.fillWidth: true
          height: 40
          Loader {
            id: realTimeFactorLoader
            anchors.fill: parent
            property double numberValue: model.data[1]
            sourceComponent: writablePositiveNumber
            onLoaded: {
              realTimeFactorItem = realTimeFactorLoader.item
            }
          }
        }
      }
    }
  }
}
