import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3
import QtGraphicalEffects 1.12

Rectangle{
    id: main
    property alias source: root.source
    property alias text: label.text
    border.color: "white"
    border.width: 5
    radius: 10

    Label {
        id: label
        text: qsTr("Label")
        anchors.horizontalCenter: parent.horizontalCenter
        color: "#443224"
        font.pixelSize: 14
    }

    DropShadow {
        anchors.fill: root
        horizontalOffset: 3
        verticalOffset: 3
        radius: 8.0
        samples: 17
        color: "#80000000"
        source: root
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
            onClicked: quickSetup.loadFuelWorld(main.text)
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