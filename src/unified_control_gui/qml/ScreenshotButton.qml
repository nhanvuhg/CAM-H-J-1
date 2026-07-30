import QtQuick 2.15
import QtQuick.Controls 2.15

MotionButton {
    id: control

    property int countdown: 0
    property string hintText: "Chụp ảnh"

    signal captureRequested()

    enabled: countdown === 0
    hoverScale: 1.012
    pressScale: 0.99
    shadowEnabled: false
    shimmerEnabled: false
    shimmerColor: "#55d4faff"
    opacity: countdown > 0 ? 0.82 : 1.0

    Accessible.name: countdown > 0
                     ? "Chụp màn hình sau " + countdown + " giây"
                     : "Chụp màn hình sau 3 giây"

    background: Rectangle {
        radius: 6
        color: "transparent"
        border.color: "#0c1726"
        border.width: 1
        gradient: Gradient {
            orientation: Gradient.Horizontal
            GradientStop { position: 0.0; color: control.pressed ? Qt.darker("#1C4D8D", 1.15) : "#1C4D8D" }
            GradientStop { position: 1.0; color: control.pressed ? Qt.darker("#0c1726", 1.15) : "#0c1726" }
        }
    }

    contentItem: Item {
        Image {
            id: cameraIcon
            anchors.centerIn: parent
            width: 34
            height: 34
            source: "icons/camera.svg"
            fillMode: Image.PreserveAspectFit
            smooth: true
            opacity: control.countdown > 0 ? 0.22 : 1.0
        }

        Rectangle {
            anchors.centerIn: parent
            width: 30
            height: 30
            radius: 15
            visible: control.countdown > 0
            color: "#cc0d1e32"
            border.color: "#7fcdf5"
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: control.countdown
                color: "#ffffff"
                font.pixelSize: 18
                font.bold: true
            }
        }

        HoverHint {
            visible: control.hovered && control.countdown === 0
            label: control.hintText
            bc: "#0c1726"
            tc: "#d6f1ff"
        }
    }

    Timer {
        interval: 1000
        repeat: true
        running: control.countdown > 0
        onTriggered: {
            control.countdown -= 1
            if (control.countdown === 0)
                control.captureRequested()
        }
    }

    Behavior on opacity {
        NumberAnimation { duration: 120 }
    }

    onClicked: {
        if (countdown === 0)
            countdown = 3
    }
}
