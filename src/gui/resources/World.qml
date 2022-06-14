import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.3

Image {
    id: root
    source: ""
    signal clicked
    sourceSize.width: 100
    sourceSize.height: 100
    property int duration: 250
    property alias text: label.text

    MouseArea {
        anchors.fill: parent
        onClicked: root.clicked()
        onPressed: {
            glow.visible = true
            animation1.start()
            animation2.start()
        }
    }

    Rectangle {
        id: glow
        visible: false

        width: 125
        height: 125
        color: "#00000000"
        radius: 125
        scale: 1.05
        border.color: "#ffffff"
    }

    Label {
        id: label
        text: qsTr("Label")
        anchors.horizontalCenter: parent.horizontalCenter
        color: "#443224"
        font.family: Constants.fontFamily
        font.pixelSize: 14
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
