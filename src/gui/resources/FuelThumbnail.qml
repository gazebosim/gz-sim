/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.12

Rectangle{
    id: main
    property alias source: root.source
    property alias text: label.text
    property alias uploader: uploader.text
    border.color: "white"
    border.width: 5
    radius: 10

    DropShadow {
        anchors.fill: root
        horizontalOffset: 3
        verticalOffset: 3
        radius: 8.0
        samples: 17
        color: "#80000000"
        source: root
    }

    ColumnLayout{
        anchors.horizontalCenter: parent.horizontalCenter

        Rectangle{
            y: 15 
            Text {
            id: label
                text: qsTr("Label")
                color: "#443224"
                font.pixelSize: 14
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }
        Rectangle{
            Text {
                id: uploader
                text: qsTr("Label")
                color: "grey"
                font.pixelSize: 14
                anchors.horizontalCenter: parent.horizontalCenter
            }
        }
    }

    Image {
        id: root
        source: ""
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2 + 10
        width: parent.width - 40
        height: parent.height - 40
        signal clicked
        property int duration: 250

        MouseArea {
            anchors.fill: parent
            onPressed: {
                glow.visible = true
                animation1.start()
                animation2.start()
            }
            onClicked: quickStart.loadFuelWorld(main.text, main.uploader)
        }

        Rectangle {
            id: glow
            visible: false

            width: parent.width - 10
            height: parent.height - 10
            color: "#00000000"
            scale: 1.05
            border.color: "#ffffff"
        }


        PropertyAnimation {
            target: glow
            id: animation1
            duration: root.duration
            loops: 1
            from: 1.05
            to: 1.2
            property: "scale"
        }

        ParallelAnimation {
            id: animation2
            SequentialAnimation {
                PropertyAnimation {
                    target: glow
                    duration: root.duration
                    loops: 1
                    from: 0.2
                    to: 1.0
                    property: "opacity"
                }
                PropertyAnimation {
                    target: glow
                    duration: root.duration
                    loops: 1
                    from: 1.0
                    to: 0.0
                    property: "opacity"
                }

                PropertyAction {
                    target: glow
                    property: "visible"
                    value: false
                }
            }

            SequentialAnimation {
                PropertyAction {
                    target: glow
                    property: "border.width"
                    running: false
                    value: 20
                }

                PauseAnimation {
                    duration: 200
                }

                PropertyAnimation {
                    target: glow
                    duration: root.duration
                    loops: 1
                    from: 20
                    to: 10
                    property: "border.width"
                }
            }
        }
    }
}