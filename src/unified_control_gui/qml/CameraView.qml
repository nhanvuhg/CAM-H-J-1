import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    Layout.fillWidth: true
    // Natural card height; the containing camera stack may override it to
    // divide the available screen height evenly between both cameras.
    implicitHeight: width * 9 / 16 + 45
    height: implicitHeight

    property string cameraName: qsTr("Camera")
    property string topic: "/camera/image_raw"
    property string providerId: "cam"
    property int camIndex: 0

    Rectangle {
        anchors.fill: parent
        color: "#1e1e1e"
        border.color: "#163a52"
        border.width: 1
        radius: 6

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 6

            Text {
                text: cameraName + " (" + topic + ")"
                color: "#6cf"
                padding: 4
                font.pixelSize: 14
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
                horizontalAlignment: Text.AlignHCenter
            }

            Rectangle {
                id: previewBox
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#2a2a2a"
                // border.color: "#444"
                radius: 4

                Item {
                    id: aspectContainer
                    // Fit a true 16:9 viewport inside both available axes.
                    // Width-only sizing could overflow vertically and clip the
                    // second camera on displays with a shorter usable height.
                    width: Math.min(previewBox.width, previewBox.height * 16 / 9)
                    height: width * 9 / 16
                    anchors.horizontalCenter: parent.horizontalCenter
                    anchors.verticalCenter: parent.verticalCenter
                    clip: true

                    Image {
                        id: camImage
                        anchors.fill: parent
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                        source: "image://" + providerId + "/" + camIndex

                        Timer {
                            // Production capture is 25 FPS. Refresh at the same
                            // cadence so every available frame can be displayed
                            // without polling faster and uploading duplicates.
                            interval: 40
                            running: root.visible
                            repeat: true
                            onTriggered: camImage.source = "image://" + providerId + "/" + camIndex + "?" + Date.now()
                        }
                    }
                }
            }
        }
    }
}
